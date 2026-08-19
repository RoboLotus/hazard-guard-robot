from __future__ import annotations

import json
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
from .baseline_builder import BaselineCollector
from .cloud import iter_thermal_cloud
from .projection import RigidTransform, ThermalPoint
from .trend import SEVERITY, evaluate_visit, load_trend_config, read_history
from .visit import PatrolVisitAccumulator
from .voxel import AnalysisConfig, analyze_points, apply_equipment_settings, load_config


class ThermalVoxelAnalyzer(Node):
    """Apply ROI statistics and patrol-to-patrol trend decisions."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_thermal_voxel_analyzer")
        self.declare_parameter("roi_config", "")
        self.declare_parameter("baseline_path", "")
        self.declare_parameter("baseline_collection_path", "")
        self.declare_parameter("baseline_minimum_valid_visits", 10)
        self.declare_parameter("history_path", "")
        self.declare_parameter("air_temperature_topic", "")
        self.declare_parameter("oil_temperature_topic", "")
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("simulated", True)
        self._config: AnalysisConfig | None = None
        self._trend_config = None
        self._baselines: dict[str, EquipmentBaseline] = {}
        self._baseline_collector: BaselineCollector | None = None
        self._sensor_values: dict[str, float | None] = {
            "air_temperature_c": None,
            "oil_temperature_c": None,
        }
        self._history: list[dict] = []
        self._latest_result: dict[str, object] | None = None
        self._latest_header = None
        self._latest_key: tuple[int, int] | None = None
        self._last_recorded_key: tuple[int, int] | None = None
        self._visit = PatrolVisitAccumulator()
        self._baseline_path: Path | None = None
        self._baseline_collection_path: Path | None = None
        self._pending_equipment_config: tuple[dict[str, object], AnalysisConfig] | None = None

        config_path = str(self.get_parameter("roi_config").value)
        if config_path:
            try:
                self._config = load_config(config_path)
                self._trend_config = load_trend_config(config_path)
            except Exception as exc:
                self.get_logger().error(f"Could not load thermal ROI config {config_path!r}: {exc}")
        else:
            self.get_logger().warning("No ROI config supplied; point clouds will be ignored")

        baseline_value = str(
            self.get_parameter("baseline_path").value
        ).strip()
        self._baseline_path = baseline_path = (
            Path(baseline_value).expanduser() if baseline_value else None
        )
        required_equipment = (
            tuple(roi.roi_id for roi in self._config.equipment_rois)
            if self._config is not None
            else ()
        )
        if baseline_path is not None and baseline_path.exists():
            try:
                loaded = load_baselines(baseline_path)
                approved = all(
                    equipment_id in loaded
                    and loaded[equipment_id].equipment.state == "validated"
                    for equipment_id in required_equipment
                )
                if approved:
                    self._baselines = loaded
                    self.get_logger().info(
                        "Loaded approved baselines for "
                        f"{len(self._baselines)} equipment items"
                    )
                else:
                    self.get_logger().warning(
                        "Thermal baseline is incomplete or not validated; "
                        "collection remains active"
                    )
            except Exception as exc:
                self.get_logger().error(f"Could not load thermal baselines {baseline_path!r}: {exc}")

        collection_value = str(
            self.get_parameter("baseline_collection_path").value
        ).strip()
        if (
            baseline_path is not None
            and required_equipment
            and not self._baselines
            and self._trend_config is not None
        ):
            self._baseline_collection_path = collection_path = (
                Path(collection_value).expanduser()
                if collection_value
                else baseline_path.with_name(
                    baseline_path.name + ".collection"
                )
            )
            try:
                self._baseline_collector = BaselineCollector(
                    collection_path,
                    baseline_path,
                    required_equipment,
                    minimum_valid_visits=int(
                        self.get_parameter(
                            "baseline_minimum_valid_visits"
                        ).value
                    ),
                    minimum_environment_points=(
                        self._trend_config.minimum_environment_points
                    ),
                )
                activation = self._activate_collected_baseline()
                self.get_logger().info(
                    activation
                    if activation is not None
                    else self._baseline_collector.progress_message()
                )
            except Exception as exc:
                self.get_logger().error(
                    "Could not initialize thermal baseline collection: "
                    f"{exc}"
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
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._equipment_config_status_publisher = self.create_publisher(
            String, "/hazard_guard/thermal/equipment_config/status", config_qos
        )
        self.create_subscription(
            String, "/hazard_guard/thermal/equipment_config",
            self._on_equipment_config, config_qos,
        )
        self.create_subscription(PointCloud2, "/hazard_guard/thermal/points", self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(String, "/hazard_guard/thermal/inspection_control", self._on_inspection_control, 10)
        self.create_service(Trigger, "/hazard_guard/thermal/start_visit", self._start_visit)
        self.create_service(Trigger, "/hazard_guard/thermal/record_visit", self._record_visit)
        self.create_service(
            Trigger,
            "/hazard_guard/thermal/baseline_status",
            self._baseline_status,
        )
        self.create_service(
            Trigger,
            "/hazard_guard/thermal/approve_baseline",
            self._approve_baseline,
        )
        self.create_service(
            Trigger,
            "/hazard_guard/thermal/reset_baseline_collection",
            self._reset_baseline_collection,
        )
        air_topic = str(self.get_parameter("air_temperature_topic").value).strip()
        oil_topic = str(self.get_parameter("oil_temperature_topic").value).strip()
        if air_topic:
            self.create_subscription(Temperature, air_topic, self._on_air_temperature, qos_profile_sensor_data)
        if oil_topic:
            self.create_subscription(Temperature, oil_topic, self._on_oil_temperature, qos_profile_sensor_data)
        self._history_timer = self.create_timer(1.0, self._publish_history_snapshot)
        self._publish_equipment_config_status("ready")

    def _publish_equipment_config_status(
        self, state: str, error: str | None = None
    ) -> None:
        if self._config is None:
            return
        counts = (
            self._baseline_collector.counts()
            if self._baseline_collector is not None
            else {}
        )
        target = (
            self._baseline_collector.minimum_valid_visits
            if self._baseline_collector is not None
            else int(self.get_parameter("baseline_minimum_valid_visits").value)
        )
        equipment = []
        for roi in self._config.equipment_rois:
            if roi.roi_id in self._baselines:
                baseline_state = "active"
                sample_count = target
            elif self._baseline_collector is not None:
                baseline_state = "collecting"
                sample_count = int(counts.get(roi.roi_id, 0))
            else:
                baseline_state = "unavailable"
                sample_count = 0
            equipment.append(
                {
                    "id": roi.roi_id,
                    "display_name": roi.display_name or roi.roi_id,
                    "baseline_state": baseline_state,
                    "baseline_sample_count": sample_count,
                    "baseline_sample_target": target,
                }
            )
        payload: dict[str, object] = {"state": state, "equipment": equipment}
        if error:
            payload["error"] = error
        self._publish_json(self._equipment_config_status_publisher, payload)

    def _on_equipment_config(self, message: String) -> None:
        try:
            document = json.loads(message.data)
            if not isinstance(document, dict) or self._config is None:
                raise ValueError("equipment configuration must be a JSON object")
            candidate = apply_equipment_settings(self._config, document)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Rejected equipment configuration: {exc}")
            self._publish_equipment_config_status("rejected", str(exc))
            return
        if self._visit.active:
            self._pending_equipment_config = (document, candidate)
            self._publish_equipment_config_status("pending")
            return
        self._apply_equipment_config(document, candidate)

    def _apply_equipment_config(
        self, document: dict[str, object], candidate: AnalysisConfig
    ) -> None:
        del document
        old_signature = tuple(
            (roi.roi_id, roi.minimum, roi.maximum)
            for roi in self._config.equipment_rois
        ) if self._config is not None else ()
        new_signature = tuple(
            (roi.roi_id, roi.minimum, roi.maximum)
            for roi in candidate.equipment_rois
        )
        topology_changed = old_signature != new_signature
        self._config = candidate
        if topology_changed:
            self._baselines = {}
            self._history = []
            self._baseline_collector = None
            if self._baseline_path is not None and self._trend_config is not None:
                collection_path = self._baseline_collection_path
                if collection_path is None:
                    collection_path = self._baseline_path.with_name(
                        self._baseline_path.name + ".collection"
                    )
                    self._baseline_collection_path = collection_path
                try:
                    self._baseline_collector = BaselineCollector(
                        collection_path,
                        self._baseline_path,
                        tuple(roi.roi_id for roi in candidate.equipment_rois),
                        minimum_valid_visits=int(
                            self.get_parameter("baseline_minimum_valid_visits").value
                        ),
                        minimum_environment_points=(
                            self._trend_config.minimum_environment_points
                        ),
                    )
                    self._baseline_collector.reset()
                except (OSError, ValueError) as exc:
                    self._baseline_collector = None
                    self.get_logger().error(
                        f"Could not reset baseline collection after ROI change: {exc}"
                    )
        self._pending_equipment_config = None
        self.get_logger().info(
            f"Applied settings for {len(candidate.equipment_rois)} equipment items"
        )
        self._publish_equipment_config_status("applied")

    def _apply_pending_equipment_config(self) -> None:
        if self._pending_equipment_config is None:
            return
        document, candidate = self._pending_equipment_config
        self._apply_equipment_config(document, candidate)

    def _on_air_temperature(self, message: Temperature) -> None:
        self._sensor_values["air_temperature_c"] = float(message.temperature)

    def _on_oil_temperature(self, message: Temperature) -> None:
        self._sensor_values["oil_temperature_c"] = float(message.temperature)

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
            sensor_values=self._sensor_values,
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
            if equipment_id:
                self._visit.focus(equipment_id)
        elif action == "clear_focus":
            self._visit.focus(None)

    def _start_visit(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._apply_pending_equipment_config()
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
        if self._baseline_collector is not None and not self._baselines:
            try:
                update = self._baseline_collector.observe(result)
                activation = self._activate_collected_baseline()
                if activation is not None:
                    response.message += "; " + activation
                    self.get_logger().info(activation)
                elif (
                    update.get("accepted")
                    or update.get("paused")
                    or update.get("resumed")
                ):
                    progress = self._baseline_collector.progress_message()
                    response.message += "; " + progress
                    self.get_logger().info(progress)
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().error(
                    "Could not update thermal baseline collection: "
                    f"{exc}"
                )
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
        self._publish_equipment_config_status("applied")
        self._apply_pending_equipment_config()
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

    def _activate_collected_baseline(self) -> str | None:
        if (
            self._baseline_collector is None
            or self._baselines
            or not self._baseline_collector.activate_if_ready()
        ):
            return None
        self._baselines = load_baselines(
            self._baseline_collector.baseline_path
        )
        return (
            "Automatically activated thermal baselines for "
            f"{len(self._baselines)} equipment items"
        )

    def _baseline_status(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        response.success = True
        if self._baselines:
            response.message = (
                "Validated thermal baseline is active for "
                f"{len(self._baselines)} equipment items"
            )
        elif self._baseline_collector is not None:
            response.message = self._baseline_collector.progress_message()
        else:
            response.success = False
            response.message = (
                "Thermal baseline collection is not configured"
            )
        return response

    def _approve_baseline(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self._baselines:
            response.success = False
            response.message = (
                "A validated thermal baseline is already active"
            )
            return response
        if self._baseline_collector is None:
            response.success = False
            response.message = (
                "Thermal baseline collection is not configured"
            )
            return response
        if not self._baseline_collector.ready:
            response.success = False
            response.message = self._baseline_collector.progress_message()
            return response
        try:
            self._baseline_collector.approve()
            self._baselines = load_baselines(
                self._baseline_collector.baseline_path
            )
            response.success = True
            response.message = (
                "Approved and activated thermal baselines for "
                f"{len(self._baselines)} equipment items"
            )
        except (OSError, ValueError, KeyError) as exc:
            response.success = False
            response.message = f"Could not approve thermal baseline: {exc}"
        return response

    def _reset_baseline_collection(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self._baselines:
            response.success = False
            response.message = (
                "Cannot reset while a validated baseline is active"
            )
        elif self._baseline_collector is None:
            response.success = False
            response.message = (
                "Thermal baseline collection is not configured"
            )
        else:
            self._baseline_collector.reset()
            response.success = True
            response.message = self._baseline_collector.progress_message()
        return response

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
