from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from hazard_guard_interfaces.msg import HazardDetection
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Temperature
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .baseline import EquipmentBaseline, load_baselines
from .cloud import iter_thermal_cloud
from .projection import RigidTransform, ThermalPoint
from .trend import SEVERITY, evaluate_visit, load_trend_config, read_history
from .visit import PatrolVisitAccumulator
from .voxel import AnalysisConfig, analyze_points, load_config


class ThermalVoxelAnalyzer(Node):
    """Apply ROI statistics and patrol-to-patrol trend decisions."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_thermal_voxel_analyzer")
        self.declare_parameter("roi_config", "")
        self.declare_parameter("baseline_path", "")
        self.declare_parameter("history_path", "")
        self.declare_parameter("air_temperature_topic", "")
        self.declare_parameter("oil_temperature_topic", "")
        self.declare_parameter("sensor_timeout_sec", 5.0)
        self.declare_parameter("required_frame_id", "")
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("simulated", True)
        self._config: AnalysisConfig | None = None
        self._trend_config = None
        self._baselines: dict[str, EquipmentBaseline] = {}
        self._sensor_values: dict[str, float | None] = {
            "air_temperature_c": None,
            "oil_temperature_c": None,
        }
        self._sensor_updated_ns: dict[str, int | None] = {
            "air_temperature_c": None,
            "oil_temperature_c": None,
        }
        self._history: list[dict] = []
        self._latest_result: dict[str, object] | None = None
        self._latest_header = None
        self._latest_key: tuple[int, int] | None = None
        self._last_recorded_key: tuple[int, int] | None = None
        self._visit = PatrolVisitAccumulator()

        config_path = str(self.get_parameter("roi_config").value)
        if config_path:
            try:
                self._config = load_config(config_path)
                self._trend_config = load_trend_config(config_path)
                required_frame = str(
                    self.get_parameter("required_frame_id").value
                ).strip()
                if required_frame and self._config.frame_id != required_frame:
                    raise ValueError(
                        "thermal ROI frame must be "
                        f"{required_frame!r}, got {self._config.frame_id!r}"
                    )
            except Exception as exc:
                self._config = None
                self._trend_config = None
                self.get_logger().error(f"Could not load thermal ROI config {config_path!r}: {exc}")
        else:
            self.get_logger().warning("No ROI config supplied; point clouds will be ignored")

        baseline_path = str(self.get_parameter("baseline_path").value).strip()
        if baseline_path:
            try:
                self._baselines = load_baselines(baseline_path)
                self.get_logger().info(f"Loaded approved baselines for {len(self._baselines)} equipment items")
            except Exception as exc:
                self.get_logger().error(f"Could not load thermal baselines {baseline_path!r}: {exc}")

        if (
            self._config is not None
            and not bool(self.get_parameter("simulated").value)
        ):
            required = {
                roi.roi_id
                for roi in self._config.equipment_rois
                if roi.threshold_mode == "baseline_primary"
            }
            missing = sorted(required.difference(self._baselines))
            if missing:
                self.get_logger().warning(
                    "Production baseline is missing for: "
                    + ", ".join(missing)
                    + "; those equipment decisions will remain WATCH"
                )

        history_path = str(self.get_parameter("history_path").value)
        if history_path and self._trend_config is not None:
            try:
                self._history = read_history(history_path, self._trend_config.history_window_visits)
                self.get_logger().info(f"Loaded {len(self._history)} completed thermal patrol visits")
            except OSError as exc:
                self.get_logger().warning(f"Could not load thermal history: {exc}")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._analysis_publisher = self.create_publisher(String, "/hazard_guard/thermal/analysis", 10)
        self._trend_publisher = self.create_publisher(String, "/hazard_guard/thermal/trend", 10)
        detection_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._detection_publisher = self.create_publisher(HazardDetection, "/hazard_guard/thermal_detections", detection_qos)
        self.create_subscription(PointCloud2, "/hazard_guard/thermal/points", self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(String, "/hazard_guard/thermal/inspection_control", self._on_inspection_control, 10)
        self.create_service(Trigger, "/hazard_guard/thermal/start_visit", self._start_visit)
        self.create_service(Trigger, "/hazard_guard/thermal/record_visit", self._record_visit)
        air_topic = str(self.get_parameter("air_temperature_topic").value).strip()
        oil_topic = str(self.get_parameter("oil_temperature_topic").value).strip()
        if air_topic:
            self.create_subscription(Temperature, air_topic, self._on_air_temperature, qos_profile_sensor_data)
        if oil_topic:
            self.create_subscription(Temperature, oil_topic, self._on_oil_temperature, qos_profile_sensor_data)
        self._history_timer = self.create_timer(1.0, self._publish_history_snapshot)

    def _on_air_temperature(self, message: Temperature) -> None:
        self._store_sensor_value("air_temperature_c", message.temperature)

    def _on_oil_temperature(self, message: Temperature) -> None:
        self._store_sensor_value("oil_temperature_c", message.temperature)

    def _store_sensor_value(self, name: str, value: float) -> None:
        numeric = float(value)
        if not math.isfinite(numeric):
            self.get_logger().warning(
                f"Ignored non-finite {name}", throttle_duration_sec=5.0
            )
            return
        self._sensor_values[name] = numeric
        self._sensor_updated_ns[name] = self.get_clock().now().nanoseconds

    def _fresh_sensor_values(self) -> dict[str, float | None]:
        timeout_sec = max(
            0.0, float(self.get_parameter("sensor_timeout_sec").value)
        )
        timeout_ns = int(timeout_sec * 1_000_000_000)
        now_ns = self.get_clock().now().nanoseconds
        fresh: dict[str, float | None] = {}
        for name, value in self._sensor_values.items():
            updated_ns = self._sensor_updated_ns[name]
            age_ns = now_ns - updated_ns if updated_ns is not None else None
            fresh[name] = (
                value
                if value is not None
                and age_ns is not None
                and 0 <= age_ns <= timeout_ns
                else None
            )
        return fresh

    def _publish_history_snapshot(self) -> None:
        """Restore the last patrol decisions in the Web UI after a restart."""
        self._history_timer.cancel()
        if not self._history or self._config is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._config.frame_id
        latest = self._history[-1]
        self._publish_json(self._trend_publisher, latest)
        if bool(self.get_parameter("publish_detections").value):
            self._publish_detections(header, latest)

    @staticmethod
    def _rigid_transform(message: object) -> RigidTransform:
        translation = message.transform.translation
        rotation = message.transform.rotation
        return RigidTransform(
            tx=translation.x, ty=translation.y, tz=translation.z,
            qx=rotation.x, qy=rotation.y, qz=rotation.z, qw=rotation.w,
        )

    def _evaluate(self, current: dict[str, object]) -> dict[str, object]:
        assert self._trend_config is not None
        return evaluate_visit(
            current,
            self._history,
            self._trend_config,
            baselines=self._baselines,
            sensor_values=self._fresh_sensor_values(),
            simulated=bool(self.get_parameter("simulated").value),
        )

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if self._config is None or self._trend_config is None:
            return
        source_frame = cloud.header.frame_id
        if not source_frame:
            self.get_logger().warning("Thermal cloud frame is missing", throttle_duration_sec=5.0)
            return
        try:
            if source_frame == self._config.frame_id:
                target_from_source = RigidTransform()
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._config.frame_id,
                    source_frame,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=0.15),
                )
                target_from_source = self._rigid_transform(transform)
            transformed = []
            for point in iter_thermal_cloud(cloud):
                x, y, z = target_from_source.apply(point.x, point.y, point.z)
                transformed.append(ThermalPoint(
                    x=x, y=y, z=z,
                    temperature_c=point.temperature_c,
                    confidence=point.confidence,
                    pixel_u=point.pixel_u,
                    pixel_v=point.pixel_v,
                ))
            current = analyze_points(
                transformed,
                self._config,
                simulated=bool(self.get_parameter("simulated").value),
            )
            current["stamp"] = {"sec": int(cloud.header.stamp.sec), "nanosec": int(cloud.header.stamp.nanosec)}
            current["recorded_at_unix_sec"] = time.time()
            self._visit.add(current)
            result = self._evaluate(current)
        except Exception as exc:
            self.get_logger().warning(f"Thermal voxel analysis skipped: {exc}", throttle_duration_sec=5.0)
            return

        self._latest_result = result
        self._latest_header = cloud.header
        self._latest_key = (int(cloud.header.stamp.sec), int(cloud.header.stamp.nanosec))
        self._publish_json(self._analysis_publisher, result)
        if bool(self.get_parameter("publish_detections").value):
            self._publish_detections(cloud.header, result)

    def _on_inspection_control(self, message: String) -> None:
        try:
            command = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignored malformed thermal inspection control")
            return
        if not isinstance(command, dict):
            return
        action = str(command.get("action", ""))
        if action == "focus_equipment":
            equipment_id = str(command.get("equipment_id", "")).strip()
            configured = {
                roi.roi_id for roi in self._config.equipment_rois
            } if self._config is not None else set()
            if equipment_id and equipment_id in configured:
                self._visit.focus(equipment_id)
            elif equipment_id:
                self._visit.focus(None)
                self.get_logger().error(
                    f"Ignored unknown thermal equipment_id {equipment_id!r}"
                )
        elif action == "clear_focus":
            self._visit.focus(None)

    def _start_visit(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._visit.start()
        response.success = True
        response.message = "Started a new thermal patrol visit"
        return response

    @staticmethod
    def _payload(result: dict[str, object]) -> str:
        return json.dumps(result, ensure_ascii=False, separators=(",", ":"), allow_nan=False)

    def _publish_json(self, publisher, result: dict[str, object]) -> None:
        message = String()
        message.data = self._payload(result)
        publisher.publish(message)

    def _record_visit(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._latest_header is None or not self._visit.active:
            response.success = False
            response.message = "No active thermal patrol visit is available"
            return response
        if not self._visit.equipment_ids:
            response.success = False
            response.message = "The patrol visit contains no focused equipment frames"
            return response
        stamp = {"sec": int(self._latest_header.stamp.sec), "nanosec": int(self._latest_header.stamp.nanosec)}
        current = self._visit.finalize(stamp)
        current["recorded_at_unix_sec"] = time.time()
        if self._config is not None:
            current["frame_id"] = self._config.frame_id
            current["schema_version"] = self._config.schema_version
        if self._trend_config is None:
            response.success = False
            response.message = "Thermal trend configuration is not available"
            return response
        result = self._evaluate(current)
        payload = self._payload(result)
        response.success, response.message = self._append_history(payload)
        if not response.success:
            return response
        self._history.append(result)
        self._history = self._history[-self._trend_config.history_window_visits:]
        self._last_recorded_key = self._latest_key
        self._visit.active = False
        self._visit.focus_equipment_id = None
        self._publish_json(self._trend_publisher, result)
        if bool(self.get_parameter("publish_detections").value):
            self._publish_detections(self._latest_header, result)
        summaries = result.get("trend_analysis", {})
        states = []
        if isinstance(summaries, dict):
            for item in summaries.get("equipment", []):
                if isinstance(item, dict) and item.get("status") != "normal":
                    states.append(f"{item['equipment_id']}={item['status']}")
        if states:
            response.message += "; " + ", ".join(states)
        return response

    def _append_history(self, payload: str) -> tuple[bool, str]:
        history_value = str(self.get_parameter("history_path").value)
        if not history_value:
            return False, "history_path is empty"
        try:
            path = Path(history_value).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(payload)
                stream.write("\n")
            return True, f"Recorded thermal patrol visit in {path}"
        except OSError as exc:
            self.get_logger().warning(f"Could not append thermal history: {exc}", throttle_duration_sec=10.0)
            return False, f"Could not append thermal history: {exc}"

    def _publish_detections(self, header, result: dict[str, object]) -> None:
        if self._config is None:
            return
        for equipment in result.get("equipment", []):
            if not isinstance(equipment, dict):
                continue
            candidates = []
            for voxel in equipment.get("voxels", []):
                if not isinstance(voxel, dict):
                    continue
                decision = voxel.get("trend_analysis", {})
                if not isinstance(decision, dict):
                    continue
                status = str(decision.get("status", "normal"))
                p95 = float(voxel["p95_temperature_c"])
                peak = float(voxel.get("max_temperature_c", p95))
                reported_temperature = peak if bool(decision.get("critical_max")) else p95
                candidates.append((SEVERITY.get(status, 0), reported_temperature, voxel, status))
            if not candidates:
                continue
            _, temperature, hottest, status = max(candidates, key=lambda item: (item[0], item[1]))
            center = hottest["center"]
            decision = hottest["trend_analysis"]
            equipment_id = str(equipment["equipment_id"])
            detection = HazardDetection()
            detection.stamp = header.stamp
            detection.frame_id = self._config.frame_id
            detection.detection_id = f"thermal-{equipment_id}"
            detection.x = float(center[0])
            detection.y = float(center[1])
            detection.z = float(center[2])
            detection.temperature_c = temperature
            minimum_points = max(1, self._config.min_points_per_voxel)
            evidence = min(1.0, float(hottest["point_count"]) / (minimum_points * 2.0))
            visits = max(1, int(decision.get("visit_count", 1)))
            detection.confidence = min(1.0, evidence * (0.7 + 0.1 * visits))
            detection.radius_m = self._config.voxel_size_m * 0.5
            reason = str(decision.get("reason", "within_expected_range"))
            # Keep this four-part protocol stable for the existing Web backend/UI.
            detection.source = f"thermal_trend:{equipment_id}:{status}:{reason}"
            detection.simulated = bool(self.get_parameter("simulated").value)
            self._detection_publisher.publish(detection)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThermalVoxelAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
