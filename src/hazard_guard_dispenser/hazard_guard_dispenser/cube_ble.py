#!/usr/bin/env python3
"""
cube_ble.py — 비콘 큐브 BLE 연결 관리 (RoboLotus)

젯슨이 큐브 3개를 찾아 연결해두고 유지한다.
배출 순간에는 이미 뚫린 연결로 1바이트만 보내면 되므로 빠르다.
큐브를 개별 지정하지 않는다. 전부에게 보내고, 떨어진 놈이 스스로 판별한다.
큐브가 "떨어졌다"고 보고하면 나머지에 즉시 CANCEL을 보낸다 (선착순 확정).

단독 테스트:  python3 cube_ble.py
"""

import asyncio
import json
import logging
import os
import threading
import time
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

# 큐브 펌웨어(BeaconCube.ino)와 글자 하나까지 같아야 한다
SERVICE_UUID = "7f4a0001-9c2b-4d3e-8a1f-6b0c5d2e9f31"
COMMAND_UUID = "7f4a0002-9c2b-4d3e-8a1f-6b0c5d2e9f31"
REPORT_UUID  = "7f4a0003-9c2b-4d3e-8a1f-6b0c5d2e9f31"
BATTERY_UUID = "7f4a0004-9c2b-4d3e-8a1f-6b0c5d2e9f31"

CMD_ARM    = b"A"
CMD_CANCEL = b"C"

RPT_NONE       = 0
RPT_DROPPED    = 1
RPT_SHAKEN_OFF = 2
RPT_AUTO_OFF   = 3
RPT_LOW_BATT   = 4

RPT_NAME = {
    RPT_DROPPED:    "DROPPED (떨어짐)",
    RPT_SHAKEN_OFF: "SHAKEN_OFF (흔들어서 끔)",
    RPT_AUTO_OFF:   "AUTO_OFF (자동 소등)",
    RPT_LOW_BATT:   "LOW_BATT (배터리 부족 소등)",
}


class CubeLink:
    def __init__(self, expected_cubes=3, scan_seconds=5.0,
                 rescan_interval=15.0, partial_rescan_interval=300.0,
                 battery_refresh_interval=30.0, logger=None,
                 installed_state_path=None):
        self.expected = expected_cubes
        self.scan_seconds = scan_seconds
        self.rescan_interval = rescan_interval
        self.partial_rescan_interval = max(
            float(partial_rescan_interval), float(rescan_interval)
        )
        self.battery_refresh_interval = max(
            float(battery_refresh_interval), 1.0
        )
        self.log = logger or logging.getLogger("CubeLink")

        self._loop = None
        self._thread = None
        self._clients = {}
        self._connected_since = {}
        self._lock = threading.Lock()
        self._running = False
        self._last_scan_monotonic = 0.0
        self._last_battery_refresh_monotonic = 0.0

        # Battery notifications arrive on the asyncio BLE thread while ROS
        # status publication reads them from an executor thread. Keep the
        # readings behind the same lock as the client registry so callers
        # never iterate a dictionary that is being mutated concurrently.
        self._battery = {}
        self._installed_state_path = (
            Path(installed_state_path).expanduser()
            if installed_state_path else None
        )
        self._installed_persistence_ok = True
        self._installed = self._load_installed()
        self._unavailable = set()
        self._drop_event = threading.Event()
        self._drop_addr = None
        self._arm_invalidated = threading.Event()
        self._arm_state = "idle"
        self.on_cube_off = None
        self.on_status_change = None

    def _load_installed(self):
        path = self._installed_state_path
        if path is None or not path.exists():
            return set()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            values = payload.get("installed", []) if isinstance(payload, dict) else []
            return {str(value) for value in values if str(value).strip()}
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._installed_persistence_ok = False
            self._warn(f"설치 비콘 상태를 읽지 못해 안전하게 비활성화합니다: {exc}")
            return set()

    def _persist_installed_locked(self):
        path = self._installed_state_path
        if path is None:
            return
        try:
            path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(
                    {"installed": sorted(self._installed)},
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            os.replace(temporary, path)
            self._installed_persistence_ok = True
        except OSError:
            self._installed_persistence_ok = False
            raise

    def reset_installed(self, address=None):
        """Maintenance-only reset after a physical beacon is reloaded."""
        with self._lock:
            if address is None:
                self._installed.clear()
            else:
                self._installed.discard(str(address))
            self._persist_installed_locked()
        self._notify_status_change()

    def mark_installed(self, address):
        """Persist a BLE drop winner before it can be considered again."""
        with self._lock:
            self._installed.add(str(address))
            self._persist_installed_locked()
        self._notify_status_change()

    def installed_persistence_ok(self):
        with self._lock:
            return bool(self._installed_persistence_ok)

    # ---------------- 외부에서 부르는 것 ----------------
    def start(self):
        if not BLEAK_AVAILABLE:
            self._warn("bleak 없음. BLE 꺼진 상태로 동작. pip3 install bleak")
            return
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._info("큐브 BLE 관리 시작")

    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._disconnect_all(), self._loop)
            time.sleep(0.5)
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._info("큐브 BLE 관리 종료")

    def connected_count(self):
        with self._lock:
            return sum(1 for c in self._clients.values() if c.is_connected)

    def arm_all(
        self, repeat=2, interval=0.05, allowed_addresses=None
    ):
        """배출 직전 호출. 반환값 = 신호가 전달된 큐브 수."""
        self._drop_event.clear()
        self._drop_addr = None
        self._arm_invalidated.clear()
        with self._lock:
            self._arm_state = "arming"

        if not BLEAK_AVAILABLE or not self._loop:
            self._warn("BLE 사용 불가. ARM 미발송")
            return 0

        fut = asyncio.run_coroutine_threadsafe(
            self._send_all(
                CMD_ARM,
                repeat,
                interval,
                allowed_addresses=allowed_addresses,
            ),
            self._loop,
        )
        try:
            n = fut.result(timeout=3.0)
            with self._lock:
                if n > 0 and not self._arm_invalidated.is_set():
                    self._arm_state = "armed"
            self._info(f"ARM 발송 완료: {n}대")
            return n
        except Exception as e:
            self._error(f"ARM 발송 실패: {e}")
            return 0

    def wait_for_drop(self, timeout=2.5):
        """낙하 보고 대기. 보고한 큐브 주소 또는 None."""
        if self._drop_event.wait(timeout):
            return self._drop_addr
        return None

    def arm_is_valid(self):
        with self._lock:
            return (
                not self._arm_invalidated.is_set()
                and self._arm_state == "armed"
            )

    def begin_actuation(self):
        """Atomically claim the armed generation before moving the servo."""
        with self._lock:
            if self._arm_invalidated.is_set() or self._arm_state != "armed":
                return False
            self._arm_state = "actuating"
            return True

    def finish_actuation(self):
        with self._lock:
            self._arm_state = "idle"

    def cancel_all(self, invalidate_arm=True):
        if invalidate_arm:
            self._arm_invalidated.set()
            with self._lock:
                if self._arm_state != "actuating":
                    self._arm_state = "invalidated"
        if not BLEAK_AVAILABLE or not self._loop:
            return 0
        fut = asyncio.run_coroutine_threadsafe(
            self._send_all(CMD_CANCEL, 1, 0), self._loop)
        try:
            return fut.result(timeout=3.0)
        except Exception:
            return 0

    # ---------------- 내부 (비동기) ----------------
    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.create_task(self._maintain())
        try:
            self._loop.run_forever()
        finally:
            self._loop.close()

    async def _maintain(self):
        while self._running:
            try:
                now = time.monotonic()
                if self._scan_due(now):
                    await self._scan_and_connect()
                    self._last_scan_monotonic = time.monotonic()
                if self._battery_refresh_due(now):
                    await self._refresh_batteries()
                    self._last_battery_refresh_monotonic = time.monotonic()
            except Exception as e:
                self._error(f"BLE 탐색 오류: {e}")
            await asyncio.sleep(min(self.rescan_interval, 5.0))

    def _scan_due(self, now=None):
        """Return whether discovery is safe and due.

        BlueZ discovery can destabilize an already connected low-power cube
        on the Jetson adapter. Discover immediately when every cube is gone,
        but back off aggressively while the partial-connection policy already
        provides at least one usable confirmation channel. Never scan while an
        approved actuation owns the BLE link.
        """
        current = time.monotonic() if now is None else float(now)
        with self._lock:
            connected = sum(
                1 for client in self._clients.values() if client.is_connected
            )
            arm_state = self._arm_state
        if arm_state in {"arming", "armed", "actuating"}:
            return False
        interval = (
            self.rescan_interval
            if connected == 0
            else self.partial_rescan_interval
        )
        return (
            connected < self.expected
            and current - self._last_scan_monotonic >= interval
        )

    def _battery_refresh_due(self, now=None):
        current = time.monotonic() if now is None else float(now)
        with self._lock:
            arm_state = self._arm_state
        return (
            arm_state == "idle"
            and current - self._last_battery_refresh_monotonic
            >= self.battery_refresh_interval
        )

    async def _refresh_batteries(self):
        """Re-read battery characteristics for every connected cube.

        Some beacon firmware sends a notification only when the voltage
        changes. A periodic GATT read keeps source timestamps current without
        treating an unchanged battery as disconnected or stale.
        """
        with self._lock:
            targets = [
                (address, client)
                for address, client in self._clients.items()
                if client.is_connected
            ]
        if not targets:
            return
        results = await asyncio.gather(
            *[client.read_gatt_char(BATTERY_UUID) for _, client in targets],
            return_exceptions=True,
        )
        for (address, _client), result in zip(targets, results):
            if isinstance(result, Exception):
                self._warn(f"{address}: 배터리 재조회 실패: {result}")
                continue
            self._on_battery(address, result)

    async def _scan_and_connect(self):
        """
        큐브를 찾아 연결한다.

        BleakScanner 의 service_uuids 인자는 리눅스(BlueZ)에서
        광고를 걸러버리는 경우가 있어 쓰지 않는다.
        필터 없이 전부 받아온 뒤 파이썬에서 직접 UUID를 대조한다.
        """
        self._info(f"큐브 탐색 중... ({self.connected_count()}/{self.expected})")

        try:
            found = await BleakScanner.discover(
                timeout=self.scan_seconds, return_adv=True)
        except Exception as e:
            self._error(f"스캔 실패: {e}")
            return

        target = SERVICE_UUID.lower()
        cubes = []
        for addr, (dev, adv) in found.items():
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            if target in uuids:
                cubes.append(dev)

        if not cubes:
            self._warn(f"큐브를 찾지 못함 (주변 {len(found)}대 스캔됨). "
                       "큐브 전원/펌웨어 확인")
            return

        self._info(f"큐브 후보 {len(cubes)}대 발견")

        for dev in cubes:
            with self._lock:
                already = (dev.address in self._clients and
                           self._clients[dev.address].is_connected)
            if already:
                continue
            await self._connect_one(dev)

    async def _connect_one(self, dev):
        client = BleakClient(dev.address,
                             disconnected_callback=self._on_disconnect)
        try:
            await client.connect(timeout=10.0)
            if not client.is_connected:
                return
            await client.start_notify(
                REPORT_UUID,
                lambda _s, data, addr=dev.address: self._on_report(addr, data))
            try:
                await client.start_notify(
                    BATTERY_UUID,
                    lambda _s, data, addr=dev.address: self._on_battery(addr, data))
                raw = await client.read_gatt_char(BATTERY_UUID)
                self._on_battery(dev.address, raw)
            except Exception:
                self._warn(f"{dev.address}: 배터리 특성 없음 (구버전 펌웨어)")
            with self._lock:
                self._clients[dev.address] = client
                self._connected_since[dev.address] = time.monotonic()
            self._info(f"큐브 연결됨: {dev.address} "
                       f"({self.connected_count()}/{self.expected})")
            self._notify_status_change()
        except Exception as e:
            self._warn(f"연결 실패 {dev.address}: {e}")
            try:
                await client.disconnect()
            except Exception:
                pass

    def _on_disconnect(self, client):
        addr = getattr(client, "address", "?")
        with self._lock:
            if self._clients.get(addr) is client:
                self._clients.pop(addr, None)
            self._connected_since.pop(addr, None)
            # A total disconnect must be eligible for immediate discovery;
            # the partial-connection backoff only applies while at least one
            # confirmation channel remains alive.
            if not any(item.is_connected for item in self._clients.values()):
                self._last_scan_monotonic = 0.0
        self._warn(f"큐브 끊김: {addr}. 다음 탐색 때 재연결")
        self._notify_status_change()

    def _on_report(self, address, data):
        if not data:
            return
        code = data[0]
        if code == RPT_NONE:
            return

        self._info(f"보고 수신 {address}: {RPT_NAME.get(code, code)}")

        if code == RPT_DROPPED:
            try:
                self.mark_installed(address)
            except OSError as exc:
                self._error(f"설치 비콘 상태 저장 실패: {exc}")
            if not self._drop_event.is_set():
                self._drop_addr = address
                self._drop_event.set()
                if self._loop is not None:
                    asyncio.run_coroutine_threadsafe(
                        self._cancel_others(address), self._loop)
        elif code in (RPT_SHAKEN_OFF, RPT_AUTO_OFF, RPT_LOW_BATT):
            if code == RPT_LOW_BATT:
                self._arm_invalidated.set()
                with self._lock:
                    self._unavailable.add(address)
                    if self._arm_state != "actuating":
                        self._arm_state = "invalidated"
                if self._loop is not None:
                    asyncio.run_coroutine_threadsafe(
                        self._send_all(CMD_CANCEL, 1, 0), self._loop
                    )
            if self.on_cube_off:
                try:
                    self.on_cube_off(address, code)
                except Exception as e:
                    self._error(f"소등 콜백 오류: {e}")
        self._notify_status_change()

    def _on_battery(self, address, data):
        """큐브가 보낸 배터리 값. 전압을 10배한 1바이트."""
        if not data:
            return
        volts = data[0] / 10.0
        now = time.monotonic()
        with self._lock:
            previous = self._battery.get(address)
            prev = previous[0] if previous is not None else None
            self._battery[address] = (volts, now, time.time())
        if prev is None or abs(volts - prev) >= 0.1:
            self._info(f"배터리 {address}: {volts:.1f}V ({self._pct(volts)}%)")
        if volts < 10.5:
            self._warn(f"배터리 부족 {address}: {volts:.1f}V. 충전 필요")
        self._notify_status_change()

    @staticmethod
    def _pct(volts):
        pct = (volts - 9.0) / (12.6 - 9.0) * 100.0
        return int(max(0, min(100, pct)) + 0.5)

    def battery_levels(self, stale_after=None):
        """Return an immutable battery snapshot for connected and stale cubes.

        Values are ``(voltage, percent, connected, stale)``. A disconnected
        cube remains visible as its last-known reading, but it is explicitly
        marked disconnected rather than being mistaken for an available cube.
        """
        now = time.monotonic()
        with self._lock:
            readings = dict(self._battery)
            connected = {
                address
                for address, client in self._clients.items()
                if client.is_connected
            }
        result = {}
        for address, (volts, updated_at, _updated_at_unix) in readings.items():
            stale = (
                stale_after is not None
                and stale_after >= 0
                and now - updated_at > stale_after
            )
            result[address] = (
                volts,
                self._pct(volts),
                address in connected,
                stale,
            )
        return result

    def battery_snapshot(self, stale_after=None):
        """Return JSON-friendly per-cube battery records."""
        return self.status_snapshot(stale_after=stale_after)["beacons"]

    def status_snapshot(self, stale_after=None):
        """Capture connection and battery state under one lock boundary."""
        now = time.monotonic()
        with self._lock:
            readings = dict(self._battery)
            connected = {
                address
                for address, client in self._clients.items()
                if client.is_connected
            }
            unavailable = set(self._unavailable) | set(self._installed)
            installed = set(self._installed)
        records = []
        for address in sorted(connected | set(readings) | installed):
            reading = readings.get(address)
            if reading is None:
                records.append(
                    {
                        "address": address,
                        "voltage": None,
                        "percent": None,
                        "connected": address in connected,
                        "stale": False,
                        "battery_supported": False,
                        "reported_unavailable": address in unavailable,
                        "installed": address in installed,
                        "updated_at_unix_ms": None,
                    }
                )
                continue
            volts, updated_at, updated_at_unix = reading
            stale = (
                stale_after is not None
                and stale_after >= 0
                and now - updated_at > stale_after
            )
            records.append(
                {
                    "address": address,
                    "voltage": round(volts, 2),
                    "percent": self._pct(volts),
                    "connected": address in connected,
                    "stale": stale,
                    "battery_supported": True,
                    "reported_unavailable": address in unavailable,
                    "installed": address in installed,
                    "updated_at_unix_ms": int(updated_at_unix * 1000),
                }
            )
        return {
            "connected": len(connected),
            "beacons": records,
            "installed_persistence_ok": self.installed_persistence_ok(),
        }

    def connected_addresses(self, stable_for=0.0):
        now = time.monotonic()
        minimum_age = max(0.0, float(stable_for))
        with self._lock:
            return {
                address
                for address, client in self._clients.items()
                if client.is_connected
                and now - self._connected_since.get(address, now)
                >= minimum_age
            }

    def _notify_status_change(self):
        callback = self.on_status_change
        if callback is None:
            return
        try:
            callback()
        except Exception as exc:
            self._error(f"상태 변경 콜백 오류: {exc}")

    def lowest_battery(self):
        """가장 낮은 큐브의 (주소, 전압, 잔량%). 값이 없으면 None."""
        with self._lock:
            readings = dict(self._battery)
        if not readings:
            return None
        address = min(readings, key=lambda item: readings[item][0])
        volts = readings[address][0]
        return address, volts, self._pct(volts)

    async def _cancel_others(self, winner):
        with self._lock:
            targets = [(a, c) for a, c in self._clients.items()
                       if c.is_connected and a != winner]
        if not targets:
            return
        await asyncio.gather(
            *[self._write_one(c, CMD_CANCEL) for _, c in targets],
            return_exceptions=True)
        self._info(f"나머지 {len(targets)}대에 CANCEL 발송")

    async def _send_all(
        self, payload, repeat, interval, allowed_addresses=None
    ):
        ok = 0
        for i in range(repeat):
            with self._lock:
                targets = [
                    client
                    for address, client in self._clients.items()
                    if client.is_connected
                    and (
                        payload != CMD_ARM
                        or (
                            address not in self._unavailable
                            and address not in self._installed
                        )
                    )
                    and (
                        allowed_addresses is None
                        or address in allowed_addresses
                    )
                ]
            if payload == CMD_ARM and self._arm_invalidated.is_set():
                break
            if payload == CMD_ARM and not self._installed_persistence_ok:
                self._error("설치 비콘 원장이 손상되어 ARM을 차단합니다")
                return 0
            if not targets:
                self._error("전송 가능한 큐브 없음")
                return 0
            results = await asyncio.gather(
                *[self._write_one(c, payload) for c in targets],
                return_exceptions=True)
            ok = sum(1 for r in results if r is True)
            if i < repeat - 1 and interval > 0:
                await asyncio.sleep(interval)
        return ok

    async def _write_one(self, client, payload):
        try:
            await client.write_gatt_char(COMMAND_UUID, payload, response=False)
            return True
        except Exception as e:
            self._warn(f"쓰기 실패 {client.address}: {e}")
            return False

    async def _disconnect_all(self):
        with self._lock:
            targets = list(self._clients.values())
        for c in targets:
            try:
                if c.is_connected:
                    await c.disconnect()
            except Exception:
                pass
        with self._lock:
            self._clients.clear()
            self._connected_since.clear()

    # ---------------- 로그 ----------------
    def _info(self, msg):
        if hasattr(self.log, "info"):
            self.log.info(f"[큐브BLE] {msg}")

    def _warn(self, msg):
        if hasattr(self.log, "warn") and not hasattr(self.log, "warning"):
            self.log.warn(f"[큐브BLE] {msg}")
        elif hasattr(self.log, "warning"):
            self.log.warning(f"[큐브BLE] {msg}")

    def _error(self, msg):
        if hasattr(self.log, "error"):
            self.log.error(f"[큐브BLE] {msg}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    if not BLEAK_AVAILABLE:
        print("bleak 미설치: pip3 install bleak")
        raise SystemExit(1)

    link = CubeLink(expected_cubes=3)
    link.on_cube_off = lambda addr, code: print(f"  -> {addr} 소등됨")
    link.start()

    print("큐브를 찾는 중입니다...")
    time.sleep(8)

    try:
        while True:
            print(f"\n연결된 큐브: {link.connected_count()}대")
            input("엔터를 치면 ARM 신호를 보냅니다 (Ctrl+C 종료) > ")
            n = link.arm_all()
            print(f"-> ARM {n}대에 전달됨")
            print("   큐브를 떨어뜨려 보세요. 6초 기다립니다...")
            addr = link.wait_for_drop(timeout=6.0)
            print(f"   ★ 낙하 보고: {addr}" if addr else "   ... 보고 없음")
    except KeyboardInterrupt:
        print("\n종료합니다")
    finally:
        link.stop()
