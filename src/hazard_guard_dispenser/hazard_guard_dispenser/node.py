#!/usr/bin/env python3
"""
node.py — 비콘 큐브 디스펜서 ROS2 노드 (RoboLotus)

Jetson ─USB(/dev/myserial)─ 확장보드 ─S1─ MG946R 서보
Jetson ─BLE─ 비콘 큐브 ×N

★ create_receive_threading() 을 절대 호출하지 않는다.
  백그라운드 Mcnamu_driver 가 같은 시리얼 포트를 읽고 있어서
  여기서 읽기 스레드를 띄우면 SerialException 이 난다.
  서보 제어는 쓰기 전용이라 읽기가 필요 없다.

구독  hazard_guard/dispenser/command (std_msgs/String)
        drop / home / status / angle:NN
발행  hazard_guard/dispenser/status  (std_msgs/String)
        ready / busy / dropped / jam_suspected / error:...
"""

import json
import os
import threading
import time
import uuid
import re

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .request_ledger import (
    IdempotencyConflictError,
    RequestLedger,
    RequestLedgerError,
)
from .command_policy import (
    allow_legacy_drop,
    allow_maintenance_command,
    physical_drop_block_reason,
)


REQUEST_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,96}$")

try:
    from .cube_ble import CubeLink
except ImportError:
    try:
        from cube_ble import CubeLink
    except ImportError:
        CubeLink = None

try:
    from Rosmaster_Lib import Rosmaster
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

    class Rosmaster:
        def set_pwm_servo(self, servo_id, angle):
            pass


class DispenserNode(Node):

    def __init__(self):
        super().__init__("dispenser_node")

        # --- 서보 (실측으로 확정된 값) ---
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("angle_home", 0)
        self.declare_parameter("angle_dump", 30)
        self.declare_parameter("step_deg", 3)
        self.declare_parameter("step_delay", 0.03)
        self.declare_parameter("dump_hold", 1.0)
        self.declare_parameter("home_hold", 0.5)

        # --- 큐브 BLE ---
        self.declare_parameter("use_cube_ble", True)
        self.declare_parameter("expected_cubes", 3)   # 탐색 목표 수(제한 아님)
        self.declare_parameter("arm_lead_time", 0.3)
        self.declare_parameter("arm_repeat", 2)
        self.declare_parameter("drop_report_timeout", 2.5)
        self.declare_parameter("battery_report_sec", 60.0)
        self.declare_parameter(
            "request_ledger_path",
            os.getenv(
                "HAZARD_GUARD_DISPENSER_LEDGER_PATH",
                "~/.local/state/hazard_guard/dispenser/requests.sqlite3",
            ),
        )
        self.declare_parameter("allow_legacy_unkeyed_commands", False)
        self.declare_parameter("allow_maintenance_manual_commands", False)
        self.declare_parameter("enable_physical_drop", False)
        self.declare_parameter("home_on_startup", False)
        self.declare_parameter("home_on_shutdown", False)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_timeout_sec", 0.75)
        self.declare_parameter("stop_linear_mps", 0.02)
        self.declare_parameter("stop_angular_rps", 0.05)
        self.declare_parameter("stop_hold_sec", 0.5)

        self.servo_id = self._p("servo_id")

        self.busy = False
        self.lock = threading.Lock()
        self.current_angle = self._p("angle_home")
        self.drop_count = 0
        self._motion_lock = threading.Lock()
        self._last_odom_monotonic = None
        self._stopped_since_monotonic = None
        self.request_ledger = None
        try:
            self.request_ledger = RequestLedger(self._p("request_ledger_path"))
        except RequestLedgerError as exc:
            # No durable record means the node cannot prove a retry is safe.
            self.get_logger().error(f"요청 원장 초기화 실패. 배출 차단: {exc}")

        self.bot = None
        self.hardware_ready = False
        if HARDWARE_AVAILABLE:
            try:
                self.bot = Rosmaster()
                self.hardware_ready = True
            except Exception as exc:
                self.get_logger().error(
                    f"Rosmaster 하드웨어 초기화 실패. 배출 차단: {exc}"
                )
        else:
            self.get_logger().warn(
                "Rosmaster_Lib 없음. 시뮬레이션 모드. 서보는 안 움직입니다")

        self.cube_link = None
        if self._p("use_cube_ble"):
            if CubeLink is None:
                self.get_logger().error(
                    "cube_ble.py 를 찾을 수 없음. 서보만 동작합니다")
            else:
                self.cube_link = CubeLink(
                    expected_cubes=self._p("expected_cubes"),
                    logger=self.get_logger())
                self.cube_link.on_cube_off = self._on_cube_off
                self.cube_link.start()
        else:
            self.get_logger().info("use_cube_ble=False. 서보만 동작")

        self.pub = self.create_publisher(
            String, "hazard_guard/dispenser/status", 10)
        self.batt_pub = self.create_publisher(
            String, "hazard_guard/dispenser/battery", 10)
        self.result_pub = self.create_publisher(
            String, "hazard_guard/dispenser/result", 10)
        if self.cube_link:
            self.create_timer(self._p("battery_report_sec"), self._publish_battery)
        try:
            from hazard_guard_interfaces.srv import DispenserRequestStatus

            self._request_status_service = self.create_service(
                DispenserRequestStatus,
                "hazard_guard/dispenser/request_status",
                self._on_request_status,
            )
        except ImportError:
            self._request_status_service = None
            self.get_logger().error("DispenserRequestStatus 인터페이스가 없어 결과 조회 서비스를 비활성화합니다")
        self.create_subscription(
            String, "hazard_guard/dispenser/command", self.on_command, 10)
        self.create_subscription(
            Odometry,
            self._p("odom_topic"),
            self._on_odom,
            10,
        )

        if self._p("home_on_startup") and self.hardware_ready:
            self._go_to(self._p("angle_home"), smooth=False)
        self.get_logger().info(
            f"디스펜서 준비 완료. 서보 S{self.servo_id}, "
            f"home={self._p('angle_home')} dump={self._p('angle_dump')}")
        self._publish_status()

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------- 명령 ----------------
    def on_command(self, msg):
        raw = msg.data.strip()
        try:
            payload = json.loads(raw)
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = None

        if isinstance(payload, dict):
            command = str(payload.get("command", "")).strip().lower()
            if command != "drop":
                self.get_logger().warn("JSON 명령은 drop만 지원합니다")
                return
            request_id = str(payload.get("request_id", "")).strip()
            detection_id = payload.get("detection_id")
            detection_id = str(detection_id).strip() if detection_id else None
            if not REQUEST_ID_PATTERN.fullmatch(request_id):
                self.get_logger().warn("유효하지 않은 request_id 배출 요청을 거부했습니다")
                return
            if detection_id is not None and not REQUEST_ID_PATTERN.fullmatch(detection_id):
                self.get_logger().warn("유효하지 않은 detection_id 배출 요청을 거부했습니다")
                return
            self._request_drop(request_id, detection_id)
            return

        cmd = raw.lower()

        if cmd == "drop":
            if not allow_legacy_drop(self._p("allow_legacy_unkeyed_commands")):
                self.get_logger().warn("멱등 키 없는 legacy drop 명령을 거부했습니다")
                return
            self.get_logger().warn("legacy drop 명령: 개발 전용 멱등성 우회")
            self._request_drop(f"legacy-{uuid.uuid4()}", None)
        elif cmd == "home":
            if not allow_maintenance_command(
                cmd, self._p("allow_maintenance_manual_commands")
            ):
                self.get_logger().warn("운영 모드에서는 home 정비 명령을 거부했습니다")
                return
            self._go_to(self._p("angle_home"))
            self.get_logger().info("챔버 대기 위치로 복귀")
        elif cmd == "status":
            self._publish_status()
        elif cmd == "battery":
            self._publish_battery()
        elif cmd.startswith("angle:"):
            if not allow_maintenance_command(
                cmd, self._p("allow_maintenance_manual_commands")
            ):
                self.get_logger().warn("운영 모드에서는 angle 정비 명령을 거부했습니다")
                return
            try:
                angle = int(cmd.split(":", 1)[1])
            except ValueError:
                self.get_logger().error(f"각도 형식 오류: {cmd}")
                return
            self._go_to(angle)
            self.get_logger().info(f"수동 각도 -> {self.current_angle}")
        else:
            self.get_logger().warn(f'모르는 명령: "{msg.data}"')

    def _on_cube_off(self, address, code):
        self.get_logger().info(f"큐브 소등 확인: {address}")

    def _publish_battery(self):
        if not self.cube_link:
            return
        levels = self.cube_link.battery_levels()
        if not levels:
            return
        parts = [f"{address[-5:]}={volts:.1f}V/{pct}%"
                 for address, (volts, pct) in levels.items()]
        msg = String()
        msg.data = " ".join(parts)
        self.batt_pub.publish(msg)

        low = self.cube_link.lowest_battery()
        if low and low[1] < 10.5:
            self.get_logger().warn(
                f"큐브 배터리 부족: {low[0]} {low[1]:.1f}V ({low[2]}%)")

    def _on_odom(self, message):
        now = time.monotonic()
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        linear_speed = (
            float(linear.x) ** 2
            + float(linear.y) ** 2
            + float(linear.z) ** 2
        ) ** 0.5
        angular_speed = (
            float(angular.x) ** 2
            + float(angular.y) ** 2
            + float(angular.z) ** 2
        ) ** 0.5
        stopped = (
            linear_speed <= float(self._p("stop_linear_mps"))
            and angular_speed <= float(self._p("stop_angular_rps"))
        )
        with self._motion_lock:
            self._last_odom_monotonic = now
            if stopped:
                if self._stopped_since_monotonic is None:
                    self._stopped_since_monotonic = now
            else:
                self._stopped_since_monotonic = None

    def _motion_is_stably_stopped(self):
        now = time.monotonic()
        with self._motion_lock:
            last_odom = self._last_odom_monotonic
            stopped_since = self._stopped_since_monotonic
        if last_odom is None or stopped_since is None:
            return False
        if now - last_odom > float(self._p("odom_timeout_sec")):
            return False
        return now - stopped_since >= float(self._p("stop_hold_sec"))

    # ---------------- 배출 ----------------
    def _request_drop(self, request_id, detection_id):
        with self.lock:
            if self.request_ledger is None:
                self._publish_result(
                    {
                        "request_id": request_id,
                        "detection_id": detection_id,
                        "state": "hardware_error",
                        "result_detail": "request_ledger_unavailable",
                    }
                )
                return
            try:
                record, created = self.request_ledger.claim(
                    request_id=request_id,
                    detection_id=detection_id,
                )
            except IdempotencyConflictError as exc:
                self.get_logger().error(f"멱등성 키 충돌. 배출 차단: {exc}")
                self._publish_result(
                    {
                        "request_id": request_id,
                        "detection_id": detection_id,
                        "state": "idempotency_conflict",
                        "result_detail": str(exc),
                    }
                )
                return
            except RequestLedgerError as exc:
                self.get_logger().error(f"요청 원장 기록 실패. 배출 차단: {exc}")
                self._publish_result(
                    {
                        "request_id": request_id,
                        "detection_id": detection_id,
                        "state": "hardware_error",
                        "result_detail": "request_ledger_write_failed",
                    }
                )
                return
            if not created:
                self._publish_result({**record, "duplicate": True})
                return
            block_reason = physical_drop_block_reason(
                enabled=bool(self._p("enable_physical_drop")),
                hardware_available=self.hardware_ready,
                motion_stopped=self._motion_is_stably_stopped(),
            )
            if block_reason is not None:
                state = (
                    "hardware_unavailable"
                    if block_reason == "hardware_unavailable"
                    else "safety_interlock"
                )
                record = self.request_ledger.transition(
                    request_id,
                    state,
                    result_detail=block_reason,
                    actuation_started=False,
                )
                self._publish_result(record)
                return
            if self.busy:
                record = self.request_ledger.transition(
                    request_id, "rejected_busy", result_detail="another_request_active"
                )
                self._publish_result(record)
                return
            self.busy = True
            record = self.request_ledger.transition(request_id, "dispensing")
        self._publish_result(record)
        threading.Thread(target=self._do_drop, args=(request_id,), daemon=True).start()

    def _do_drop(self, request_id):
        final_record = None
        actuation_started = False
        try:
            self._publish_status()
            self.get_logger().info("배출 시작")

            # 0) ARM — 반드시 기울이기 "전에"
            armed = 0
            if self.cube_link:
                self.get_logger().info("  0) 큐브에 ARM 발송")
                armed = self.cube_link.arm_all(repeat=self._p("arm_repeat"))
            block_reason = physical_drop_block_reason(
                enabled=True,
                hardware_available=self.hardware_ready,
                motion_stopped=self._motion_is_stably_stopped(),
                armed_count=armed,
            )
            if block_reason is not None:
                state = (
                    "rejected_no_confirmation"
                    if block_reason == "no_ble_confirmation_channel"
                    else "safety_interlock"
                )
                self.get_logger().error(f"     안전 조건 미충족: {block_reason}")
                final_record = self.request_ledger.transition(
                    request_id,
                    state,
                    result_detail=block_reason,
                    connected_cubes=(
                        self.cube_link.connected_count() if self.cube_link else 0
                    ),
                    actuation_started=False,
                )
                self._publish(state)
                if self.cube_link:
                    self.cube_link.cancel_all()
                return
            time.sleep(self._p("arm_lead_time"))

            # 1) 기울임
            self.get_logger().info("  1) 챔버 기울임")
            actuation_started = True
            self.request_ledger.transition(
                request_id, "dispensing", actuation_started=True
            )
            self._go_to(self._p("angle_dump"))

            # 2) 낙하 보고 대기
            dropped_by = None
            if self.cube_link and armed > 0:
                self.get_logger().info("  2) 낙하 보고 대기")
                self.request_ledger.transition(request_id, "waiting")
                dropped_by = self.cube_link.wait_for_drop(
                    timeout=self._p("drop_report_timeout"))
                if dropped_by:
                    self.get_logger().info(f"     배출 확인: {dropped_by}")
                else:
                    self.get_logger().warn(
                        "     보고 없음. 걸림 또는 큐브 연결 문제 의심")
            else:
                self.get_logger().info("  2) 미끄러짐 대기")
                time.sleep(self._p("dump_hold"))

            # 3) 복귀
            self.get_logger().info("  3) 챔버 복귀")
            self.request_ledger.transition(request_id, "homing")
            self._go_to(self._p("angle_home"))
            time.sleep(self._p("home_hold"))

            self.drop_count += 1
            self.get_logger().info(f"배출 완료. 누적 {self.drop_count}회")

            outcome = "succeeded" if dropped_by else "jam_suspected"
            detail = "ble_drop_confirmed" if dropped_by else "drop_report_missing"
            final_record = self.request_ledger.transition(
                request_id,
                outcome,
                result_detail=detail,
                dropped_by=dropped_by,
                actuation_started=True,
                home_recovered=True,
            )
            self._publish("dropped" if outcome == "succeeded" else outcome)

        except Exception as e:
            self.get_logger().error(f"배출 중 오류: {e}")
            home_recovered = False
            if actuation_started and self.hardware_ready:
                try:
                    self._go_to(self._p("angle_home"), smooth=False)
                    home_recovered = True
                except Exception as home_error:
                    self.get_logger().error(
                        f"오류 후 home 복귀 실패: {home_error}"
                    )
            try:
                final_record = self.request_ledger.transition(
                    request_id,
                    "jam_suspected" if actuation_started else "hardware_error",
                    result_detail=str(e),
                    actuation_started=actuation_started,
                    home_recovered=home_recovered,
                )
            except RequestLedgerError:
                pass
            self._publish(f"error:{e}")
        finally:
            with self.lock:
                self.busy = False
            if final_record is not None:
                self._publish_result(final_record)
            self._publish_status()

    # ---------------- 서보 ----------------
    def _go_to(self, target, smooth=True):
        if not self.hardware_ready or self.bot is None:
            raise RuntimeError("Rosmaster 하드웨어가 준비되지 않았습니다")
        target = max(0, min(180, int(target)))

        if not smooth:
            self.bot.set_pwm_servo(self.servo_id, target)
            self.current_angle = target
            return

        step = self._p("step_deg")
        delay = self._p("step_delay")
        direction = 1 if target > self.current_angle else -1

        angle = self.current_angle
        while angle != target:
            move = min(step, abs(target - angle))
            angle += move * direction
            self.bot.set_pwm_servo(self.servo_id, angle)
            time.sleep(delay)

        self.current_angle = target

    # ---------------- 상태 ----------------
    def _publish_status(self):
        self._publish("busy" if self.busy else "ready")

    def _publish(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def _publish_result(self, record):
        message = String()
        message.data = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        self.result_pub.publish(message)

    def _on_request_status(self, request, response):
        request_id = str(request.request_id).strip()
        if self.request_ledger is None or not REQUEST_ID_PATTERN.fullmatch(request_id):
            response.found = False
            response.record_json = ""
            return response
        try:
            record = self.request_ledger.get(request_id)
        except RequestLedgerError as exc:
            self.get_logger().error(f"요청 원장 조회 실패: {exc}")
            response.found = False
            response.record_json = ""
            return response
        response.found = record is not None
        response.record_json = (
            json.dumps(record, ensure_ascii=False, separators=(",", ":"))
            if record is not None else ""
        )
        return response

    def shutdown(self):
        if self._p("home_on_shutdown") and self.hardware_ready:
            try:
                self._go_to(self._p("angle_home"), smooth=False)
            except Exception:
                pass
        try:
            if self.cube_link:
                self.cube_link.stop()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DispenserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
