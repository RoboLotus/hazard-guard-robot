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
import logging
import threading
import time

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
                 rescan_interval=15.0, logger=None):
        self.expected = expected_cubes
        self.scan_seconds = scan_seconds
        self.rescan_interval = rescan_interval
        self.log = logger or logging.getLogger("CubeLink")

        self._loop = None
        self._thread = None
        self._clients = {}
        self._lock = threading.Lock()
        self._running = False

        # Battery notifications arrive on the asyncio BLE thread while ROS
        # status publication reads them from an executor thread. Keep the
        # readings behind the same lock as the client registry so callers
        # never iterate a dictionary that is being mutated concurrently.
        self._battery = {}
        self._drop_event = threading.Event()
        self._drop_addr = None
        self.on_cube_off = None

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

    def arm_all(self, repeat=2, interval=0.05):
        """배출 직전 호출. 반환값 = 신호가 전달된 큐브 수."""
        self._drop_event.clear()
        self._drop_addr = None

        if not BLEAK_AVAILABLE or not self._loop:
            self._warn("BLE 사용 불가. ARM 미발송")
            return 0

        fut = asyncio.run_coroutine_threadsafe(
            self._send_all(CMD_ARM, repeat, interval), self._loop)
        try:
            n = fut.result(timeout=3.0)
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

    def cancel_all(self):
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
                if self.connected_count() < self.expected:
                    await self._scan_and_connect()
            except Exception as e:
                self._error(f"BLE 탐색 오류: {e}")
            await asyncio.sleep(self.rescan_interval)

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
            self._info(f"큐브 연결됨: {dev.address} "
                       f"({self.connected_count()}/{self.expected})")
        except Exception as e:
            self._warn(f"연결 실패 {dev.address}: {e}")
            try:
                await client.disconnect()
            except Exception:
                pass

    def _on_disconnect(self, client):
        addr = getattr(client, "address", "?")
        self._warn(f"큐브 끊김: {addr}. 다음 탐색 때 재연결")

    def _on_report(self, address, data):
        if not data:
            return
        code = data[0]
        if code == RPT_NONE:
            return

        self._info(f"보고 수신 {address}: {RPT_NAME.get(code, code)}")

        if code == RPT_DROPPED:
            if not self._drop_event.is_set():
                self._drop_addr = address
                self._drop_event.set()
                asyncio.run_coroutine_threadsafe(
                    self._cancel_others(address), self._loop)
        elif code in (RPT_SHAKEN_OFF, RPT_AUTO_OFF):
            if self.on_cube_off:
                try:
                    self.on_cube_off(address, code)
                except Exception as e:
                    self._error(f"소등 콜백 오류: {e}")

    def _on_battery(self, address, data):
        """큐브가 보낸 배터리 값. 전압을 10배한 1바이트."""
        if not data:
            return
        volts = data[0] / 10.0
        now = time.monotonic()
        with self._lock:
            previous = self._battery.get(address)
            prev = previous[0] if previous is not None else None
            self._battery[address] = (volts, now)
        if prev is None or abs(volts - prev) >= 0.1:
            self._info(f"배터리 {address}: {volts:.1f}V ({self._pct(volts)}%)")
        if volts < 10.5:
            self._warn(f"배터리 부족 {address}: {volts:.1f}V. 충전 필요")

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
        for address, (volts, updated_at) in readings.items():
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

    async def _send_all(self, payload, repeat, interval):
        with self._lock:
            targets = [c for c in self._clients.values() if c.is_connected]
        if not targets:
            self._error("연결된 큐브 없음")
            return 0
        ok = 0
        for i in range(repeat):
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
