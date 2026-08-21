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

from .baseline import GasBaselineStore
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
        self.declare_parameter(
            "baseline_path",
            str(Path.home() / ".ros" / "hazard_guard" / "gas_baselines.json"),
        )
        self.declare_parameter(
            "inspection_control_topic",
            "/hazard_guard/thermal/inspection_control",
        )
        self.scenario = GasScenario.load(
            str(self.get_parameter("scenario_path").value)
        )
        self._baselines = GasBaselineStore(
            str(self.get_parameter("baseline_path").value),
            self.scenario.decision.baseline_min_visits,
        )
        self._engines: dict[tuple[bool, str], GasDecisionEngine] = {}
        self._search_context: tuple[bool, str] | None = None
        self._peak_voc_index = self.scenario.ambient.voc_index
        self._peak_x: float | None = None
        self._peak_y: float | None = None
        self._search_samples = 0
        self._search_location_count = 0
        self._last_search_position: tuple[float, float] | None = None
        self._thermal_by_equipment: dict[str, dict[str, object]] = {}
        self._last_event_signature: tuple[bool, str, str, str, str] | None = None
        self._visit_equipment_id: str | None = None
        self._visit_simulated: bool | None = None
        self._visit_readings: list[GasVector] = []
        self._visit_blocked = False
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
        self.create_subscription(
            String,
            str(self.get_parameter("inspection_control_topic").value),
            self._on_inspection_control,
            10,
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

    def _engine_for(
        self,
        equipment_id: str,
        simulated: bool,
    ) -> GasDecisionEngine:
        key = (simulated, equipment_id)
        engine = self._engines.get(key)
        if engine is None:
            _, baseline = self._baselines.status(equipment_id, simulated)
            ambient = baseline.vector if baseline is not None else self.scenario.ambient
            engine = GasDecisionEngine(
                ambient,
                self.scenario.decision,
                baseline_ready=baseline is not None,
            )
            self._engines[key] = engine
        return engine

    def _finish_baseline_visit(self) -> None:
        equipment_id = self._visit_equipment_id
        simulated = self._visit_simulated
        if (
            equipment_id
            and simulated is not None
            and self._visit_readings
            and not self._visit_blocked
        ):
            before_count, before = self._baselines.status(equipment_id, simulated)
            baseline = self._baselines.add_visit(
                equipment_id,
                simulated,
                self._visit_readings,
            )
            after_count, _ = self._baselines.status(equipment_id, simulated)
            if baseline is not None:
                self._engine_for(equipment_id, simulated).set_ambient(
                    baseline.vector
                )
                if before is None:
                    self.get_logger().info(
                        f"Gas baseline ready for {equipment_id}: "
                        f"{baseline.visit_count} visits"
                    )
            elif after_count > before_count:
                self.get_logger().info(
                    f"Gas baseline candidate for {equipment_id}: "
                    f"{after_count}/{self.scenario.decision.baseline_min_visits}"
                )
        self._visit_equipment_id = None
        self._visit_simulated = None
        self._visit_readings = []
        self._visit_blocked = False

    def _on_inspection_control(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        action = str(payload.get("action") or "")
        if action == "focus_equipment":
            equipment_id = str(payload.get("equipment_id") or "").strip()
            if not equipment_id:
                return
            if self._visit_equipment_id != equipment_id:
                self._finish_baseline_visit()
                self._visit_equipment_id = equipment_id
        elif action == "clear_focus":
            self._finish_baseline_visit()

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

    def _fresh_thermal(
        self,
        equipment_id: str,
        simulated: bool,
    ) -> dict[str, object] | None:
        evidence = self._thermal_by_equipment.get(equipment_id)
        if evidence is None or bool(evidence["simulated"]) != simulated:
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
            warmed_up_value = reading["warmed_up"]
            simulated_value = reading.get("simulated", False)
            if not isinstance(warmed_up_value, bool) or not isinstance(
                simulated_value,
                bool,
            ):
                raise ValueError("boolean gas-reading fields are malformed")
            if not all(
                math.isfinite(value)
                for value in (
                    vector.voc_index,
                    vector.co_ppm,
                    vector.co2_ppm,
                    elapsed,
                )
            ):
                raise ValueError("non-finite gas reading")
            warmed_up = warmed_up_value
            equipment_id = str(
                reading.get("equipment_id")
                or self.scenario.source.equipment_id
            )
            simulated = simulated_value
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning("Discarded malformed gas reading")
            return
        engine = self._engine_for(equipment_id, simulated)
        search_context = (simulated, equipment_id)
        if self._search_context != search_context:
            self._search_context = search_context
            self._peak_voc_index = engine.ambient.voc_index
            self._peak_x = None
            self._peak_y = None
            self._search_samples = 0
            self._search_location_count = 0
            self._last_search_position = None
        decision = engine.update(vector, elapsed, warmed_up)
        if decision.state == "normal":
            self._peak_voc_index = engine.ambient.voc_index
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

        thermal = self._fresh_thermal(equipment_id, simulated)
        voc_abnormal = decision.state in {
            "voc_watch",
            "investigating",
            "warning",
            "critical",
        }
        co_warning = engine.co_warning(vector)
        co_critical = engine.co_critical(vector)
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
        if warmed_up and self._visit_equipment_id == equipment_id:
            if self._visit_simulated is None:
                self._visit_simulated = simulated
            elif self._visit_simulated != simulated:
                self._visit_blocked = True
            self._visit_readings.append(vector)
            if decision.level != "info" or fusion.level != "normal":
                self._visit_blocked = True
        baseline_visits, baseline = self._baselines.status(
            equipment_id,
            simulated,
        )
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
            "baseline_ready": baseline is not None,
            "baseline_visit_count": baseline_visits,
            "baseline_required_visits": (
                self.scenario.decision.baseline_min_visits
            ),
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
            "simulated": simulated,
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
        signature = (
            simulated,
            equipment_id,
            decision.state,
            fusion.level,
            fusion.reason,
        )
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
