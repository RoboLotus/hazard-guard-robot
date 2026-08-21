from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from hazard_guard_interfaces.msg import HazardDetection
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .decision import GasDecisionEngine
from .fusion import fuse_risk, thermal_identity
from .model import GasScenario, GasVector


class GasDetector(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_gas_detector")
        default_config = (
            Path(get_package_share_directory("hazard_guard_gas_monitor"))
            / "config"
            / "demo_gas_scenario.json"
        )
        self.declare_parameter("scenario_path", str(default_config))
        self.scenario = GasScenario.load(
            str(self.get_parameter("scenario_path").value)
        )
        self.engine = GasDecisionEngine(
            self.scenario.ambient, self.scenario.decision
        )
        self._peak_voc_index = self.scenario.ambient.voc_index
        self._peak_x: float | None = None
        self._peak_y: float | None = None
        self._search_samples = 0
        self._search_location_count = 0
        self._last_search_position: tuple[float, float] | None = None
        self._thermal_by_equipment: dict[str, dict[str, object]] = {}
        self._last_event_signature: tuple[str, str, str] | None = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/hazard_guard/gas/status", status_qos
        )
        self._event_publisher = self.create_publisher(
            String, "/hazard_guard/gas/event", 20
        )
        self._pause_publisher = self.create_publisher(
            Bool, "/hazard_guard/gas/navigation_pause", status_qos
        )
        self._control_publisher = self.create_publisher(
            String, "/hazard_guard/gas/control", 10
        )
        self.create_subscription(
            String, "/hazard_guard/gas/reading", self._on_reading, 10
        )
        detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            # Only fuse live camera evidence. A volatile subscription accepts
            # both volatile and transient-local publishers without replaying
            # a detection retained by a previous simulation run.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            HazardDetection,
            "/hazard_guard/thermal_detections",
            self._on_thermal_detection,
            detection_qos,
        )
        self.get_logger().info("Gas early-warning detector ready")

    def _on_thermal_detection(self, message: HazardDetection) -> None:
        equipment_id, status, reason = thermal_identity(
            message.source, float(message.temperature_c)
        )
        if not equipment_id:
            return
        stamp_ns = int(message.stamp.sec) * 1_000_000_000 + int(
            message.stamp.nanosec
        )
        now_ns = self.get_clock().now().nanoseconds
        max_age = self.scenario.decision.thermal_confirmation_max_age_sec
        if stamp_ns > 0 and now_ns > 0:
            age_sec = (now_ns - stamp_ns) / 1_000_000_000
            if age_sec < -1.0 or age_sec > max_age:
                return
        self._thermal_by_equipment[equipment_id] = {
            "status": status,
            "reason": reason,
            "temperature_c": float(message.temperature_c),
            "x": float(message.x),
            "y": float(message.y),
            "frame_id": str(message.frame_id),
            "received_monotonic": time.monotonic(),
            "simulated": bool(message.simulated),
        }

    def _fresh_thermal(self, equipment_id: str) -> dict[str, object] | None:
        evidence = self._thermal_by_equipment.get(equipment_id)
        if evidence is None:
            return None
        age = time.monotonic() - float(evidence["received_monotonic"])
        if age > self.scenario.decision.thermal_confirmation_max_age_sec:
            return None
        return evidence

    def _on_reading(self, message: String) -> None:
        try:
            reading = json.loads(message.data)
            vector = GasVector(
                float(reading["voc_index"]),
                float(reading["co_ppm"]),
                float(reading["co2_ppm"]),
            )
            elapsed = float(reading["elapsed_sec"])
            warmed_up = bool(reading["warmed_up"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning("Discarded malformed gas reading")
            return
        decision = self.engine.update(vector, elapsed, warmed_up)
        if decision.state == "normal":
            self._peak_voc_index = self.scenario.ambient.voc_index
            self._peak_x = None
            self._peak_y = None
            self._search_samples = 0
            self._search_location_count = 0
            self._last_search_position = None
        elif decision.level != "info":
            self._search_samples += 1
            if vector.voc_index >= self._peak_voc_index:
                self._peak_voc_index = vector.voc_index
                self._peak_x = float(reading.get("x", 0.0))
                self._peak_y = float(reading.get("y", 0.0))
        if decision.state == "investigating":
            current_position = (
                float(reading.get("x", 0.0)),
                float(reading.get("y", 0.0)),
            )
            if self._last_search_position is None or math.dist(
                current_position, self._last_search_position
            ) >= 0.15:
                self._search_location_count += 1
                self._last_search_position = current_position

        equipment_id = str(
            reading.get("equipment_id")
            or self.scenario.source.equipment_id
        )
        thermal = self._fresh_thermal(equipment_id)
        co_delta = vector.co_ppm - self.scenario.ambient.co_ppm
        voc_abnormal = decision.state in {
            "voc_watch",
            "investigating",
            "warning",
            "critical",
        }
        co_warning = co_delta >= self.scenario.decision.co_warning_delta_ppm
        co_critical = co_delta >= self.scenario.decision.co_critical_delta_ppm
        fusion = fuse_risk(
            voc_abnormal=voc_abnormal,
            co_warning=co_warning,
            co_critical=co_critical,
            localized_voc=(
                decision.state == "investigating"
                and self._search_location_count >= 3
            ),
            thermal_status=(
                str(thermal["status"]) if thermal is not None else "normal"
            ),
            same_zone=thermal is not None,
        )
        navigation_pause = decision.navigation_pause or fusion.level in {
            "warning",
            "critical",
        }
        payload = {
            "schema_version": 1,
            "event_id": (
                f"fusion-{equipment_id}-{fusion.level}-{fusion.reason}"
            ),
            "state": decision.state,
            "gas_level": decision.level,
            "gas_reason": decision.reason,
            "level": fusion.level,
            "reason": fusion.reason,
            "title": fusion.title,
            "frame_id": reading.get("frame_id", "odom"),
            "x": reading.get("x"),
            "y": reading.get("y"),
            "voc_index": reading.get("voc_index"),
            "co_ppm": reading.get("co_ppm"),
            "co2_ppm": reading.get("co2_ppm"),
            "source_id": reading.get("source_id"),
            "source_name": reading.get("source_name"),
            "equipment_id": equipment_id,
            "visual_model_id": reading.get("visual_model_id"),
            "source_phase": reading.get("phase"),
            "navigation_pause": navigation_pause,
            "fan_on": decision.fan_on,
            "peak_voc_index": round(self._peak_voc_index, 2),
            "peak_x": self._peak_x,
            "peak_y": self._peak_y,
            "search_samples": self._search_samples,
            "search_location_count": self._search_location_count,
            "thermal_status": (
                str(thermal["status"]) if thermal is not None else "unavailable"
            ),
            "thermal_reason": (
                str(thermal["reason"]) if thermal is not None else None
            ),
            "thermal_temperature_c": (
                float(thermal["temperature_c"])
                if thermal is not None
                else None
            ),
            "thermal_x": float(thermal["x"]) if thermal is not None else None,
            "thermal_y": float(thermal["y"]) if thermal is not None else None,
            "simulated": bool(reading.get("simulated", False)),
            "elapsed_sec": elapsed,
        }
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(status)
        pause = Bool()
        pause.data = navigation_pause
        self._pause_publisher.publish(pause)
        control = String()
        control.data = json.dumps(
            {"fan_on": decision.fan_on, "reason": fusion.reason},
            ensure_ascii=False,
        )
        self._control_publisher.publish(control)
        signature = (decision.state, fusion.level, fusion.reason)
        changed = signature != self._last_event_signature
        self._last_event_signature = signature
        if changed and fusion.level != "normal":
            event = String()
            event.data = status.data
            self._event_publisher.publish(event)
            self.get_logger().warning(
                f"{payload['title']}: VOC={vector.voc_index:.1f}, "
                f"CO={vector.co_ppm:.2f} ppm, "
                f"thermal={payload['thermal_status']}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GasDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
