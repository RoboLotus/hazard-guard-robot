"""ROS node that accumulates heat on an immutable exported 3D map."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

from .cloud import create_frozen_thermal_cloud, iter_thermal_cloud
from .frozen_map import (
    FixedGeometry,
    FrozenThermalLayer,
    LocalizationStabilityGate,
    ThermalStateError,
    VoxelHashIndex,
    fixed_stride_indices,
    is_keyframe_pose,
    thermal_update_due,
)


def _stamp_nanoseconds(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _iso_timestamp(nanoseconds: int) -> str:
    if nanoseconds <= 0:
        return ""
    return datetime.fromtimestamp(
        nanoseconds / 1_000_000_000,
        tz=timezone.utc,
    ).isoformat()


class FrozenThermalMapNode(Node):
    """Match live calibrated thermal points to fixed PLY vertices only."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_frozen_thermal_map")
        self.declare_parameter("map_cloud_path", "")
        self.declare_parameter("thermal_state_path", "")
        self.declare_parameter("session_id", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter(
            "sensor_frame", "thermal_camera_optical_frame"
        )
        self.declare_parameter("input_topic", "/hazard_guard/thermal/points")
        self.declare_parameter("output_topic", "/hazard_guard/thermal/map")
        self.declare_parameter(
            "status_topic", "/hazard_guard/thermal/map/status"
        )
        self.declare_parameter("geometry_voxel_size_m", 0.03)
        self.declare_parameter("maximum_geometry_voxels", 250_000)
        self.declare_parameter("maximum_source_vertices", 1_000_000)
        self.declare_parameter("maximum_observations_per_frame", 12_000)
        self.declare_parameter("maximum_published_voxels", 100_000)
        self.declare_parameter("association_radius_m", 0.08)
        self.declare_parameter("maximum_surface_range_residual_m", 0.05)
        self.declare_parameter("minimum_match_ratio", 0.30)
        self.declare_parameter("minimum_observations_per_frame", 20)
        self.declare_parameter("temperature_ema_alpha", 0.35)
        self.declare_parameter("keyframe_translation_m", 0.10)
        self.declare_parameter("keyframe_rotation_deg", 6.0)
        self.declare_parameter("stationary_refresh_interval_sec", 5.0)
        self.declare_parameter("rejected_frame_retry_sec", 2.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("save_interval_sec", 15.0)
        self.declare_parameter("observation_tf_wait_sec", 0.50)
        self.declare_parameter("observation_processing_rate_hz", 10.0)
        self.declare_parameter("maximum_observation_age_sec", 1.0)
        self.declare_parameter("localization_stable_samples", 3)
        self.declare_parameter("localization_stable_translation_m", 0.05)
        self.declare_parameter("localization_stable_rotation_deg", 3.0)
        self.declare_parameter("color_min_c", 10.0)
        self.declare_parameter("color_max_c", 60.0)
        # Optional local alignment never publishes a TF and never modifies
        # AMCL/Nav2.  It is conservative and off unless explicitly requested.
        self.declare_parameter("enable_local_alignment", False)
        self.declare_parameter("alignment_search_radius_m", 0.08)
        self.declare_parameter("maximum_alignment_translation_m", 0.10)

        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_topic").value),
            snapshot_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            snapshot_qos,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._layer: FrozenThermalLayer | None = None
        self._last_keyframe_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._last_rejected_integration_monotonic = -math.inf
        self._last_successful_integration_monotonic = -math.inf
        self._last_observation_at_ns = 0
        self._last_published_voxel_count = 0
        self._snapshot_truncated = False
        self._map_error = ""
        self._state_error = ""
        self._persistence_allowed = True
        self._geometry_retryable = False
        self._pending_observation: tuple[PointCloud2, float] | None = None
        self._dropped_pending_observation_count = 0
        self._last_localization_sample_stamp_ns = 0
        self._publish_geometry_indices = np.empty(0, dtype=np.int32)
        self._localization_gate = LocalizationStabilityGate(
            required_samples=int(
                self.get_parameter("localization_stable_samples").value
            ),
            maximum_translation_m=float(
                self.get_parameter(
                    "localization_stable_translation_m"
                ).value
            ),
            maximum_rotation_rad=math.radians(
                float(
                    self.get_parameter(
                        "localization_stable_rotation_deg"
                    ).value
                )
            ),
        )

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._on_observation,
            qos_profile_sensor_data,
        )
        publish_rate = max(
            0.05,
            float(self.get_parameter("publish_rate_hz").value),
        )
        save_interval = max(
            1.0,
            float(self.get_parameter("save_interval_sec").value),
        )
        self.create_timer(1.0 / publish_rate, self._publish_snapshot)
        self.create_timer(save_interval, self.persist_if_dirty)
        self.create_timer(5.0, self._retry_geometry_load)
        processing_rate = max(
            1.0,
            float(
                self.get_parameter("observation_processing_rate_hz").value
            ),
        )
        self.create_timer(1.0 / processing_rate, self._process_pending_observation)
        self._try_load_geometry()
        self._publish_status()

    @property
    def _map_path(self) -> Path | None:
        value = str(self.get_parameter("map_cloud_path").value).strip()
        return Path(value).expanduser() if value else None

    @property
    def _state_path(self) -> Path | None:
        value = str(self.get_parameter("thermal_state_path").value).strip()
        return Path(value).expanduser() if value else None

    def _try_load_geometry(self) -> None:
        if self._layer is not None:
            return
        path = self._map_path
        if path is None:
            self._geometry_retryable = False
            self._map_error = "map_cloud_path_is_empty"
            self.get_logger().error(
                "Frozen thermal map is enabled without map_cloud_path"
            )
            return
        if not path.is_file():
            self._geometry_retryable = True
            self._map_error = "map_cloud_path_not_found"
            self.get_logger().warning(
                f"Waiting for fixed 3D map PLY: {path}",
                throttle_duration_sec=10.0,
            )
            return
        try:
            geometry = FixedGeometry.from_ply(
                path,
                voxel_size_m=float(
                    self.get_parameter("geometry_voxel_size_m").value
                ),
                maximum_voxels=int(
                    self.get_parameter("maximum_geometry_voxels").value
                ),
                maximum_source_vertices=int(
                    self.get_parameter("maximum_source_vertices").value
                ),
            )
            lookup_cell_size = max(
                float(self.get_parameter("association_radius_m").value),
                float(
                    self.get_parameter("alignment_search_radius_m").value
                ),
            )
            layer = FrozenThermalLayer(
                geometry,
                VoxelHashIndex(geometry, lookup_cell_size),
            )
            state_path = self._state_path
            if state_path is not None and state_path.exists():
                try:
                    layer.restore(state_path)
                    self._last_observation_at_ns = (
                        layer.latest_observation_ns
                    )
                except ThermalStateError as exc:
                    # Never overwrite a layer belonging to a different map.
                    self._state_error = str(exc)
                    self._persistence_allowed = False
                    self.get_logger().error(
                        "Thermal state was not restored and will not be "
                        f"overwritten: {exc}"
                    )
            self._layer = layer
            self._publish_geometry_indices = fixed_stride_indices(
                int(geometry.points.shape[0]),
                max(
                    1,
                    int(
                        self.get_parameter(
                            "maximum_published_voxels"
                        ).value
                    ),
                ),
            )
            self._geometry_retryable = False
            self._map_error = ""
            self.get_logger().info(
                "Frozen thermal geometry ready: "
                f"{geometry.points.shape[0]} voxels, "
                f"fingerprint={geometry.fingerprint[:12]}..."
            )
        except Exception as exc:
            # Parse errors and configured size-limit failures are permanent
            # for this process.  Re-reading a multi-gigabyte bad map every
            # five seconds would compete with navigation for CPU and I/O.
            self._geometry_retryable = False
            self._map_error = str(exc)
            self.get_logger().error(f"Failed to load fixed 3D map: {exc}")

    def _retry_geometry_load(self) -> None:
        if self._layer is None and self._geometry_retryable:
            self._try_load_geometry()
            self._publish_status()

    def _sample_observations(
        self,
        message: PointCloud2,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        total_records = max(0, int(message.width) * int(message.height))
        maximum = max(
            1,
            int(self.get_parameter("maximum_observations_per_frame").value),
        )
        step = max(1, math.ceil(total_records / maximum))
        selected = [
            point
            for index, point in enumerate(iter_thermal_cloud(message))
            if index % step == 0
        ]
        if not selected:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty(0, dtype=np.float32),
                np.empty(0, dtype=np.float32),
            )
        points = np.asarray(
            [(point.x, point.y, point.z) for point in selected],
            dtype=np.float32,
        )
        temperatures = np.fromiter(
            (point.temperature_c for point in selected),
            dtype=np.float32,
            count=len(selected),
        )
        confidence = np.fromiter(
            (point.confidence for point in selected),
            dtype=np.float32,
            count=len(selected),
        )
        return points, temperatures, confidence

    def _lookup_observation_context(
        self,
        message: PointCloud2,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if _stamp_nanoseconds(message.header.stamp) <= 0:
            raise ValueError("thermal observation has a zero timestamp")
        lookup_time = Time.from_msg(message.header.stamp)
        transform = self._tf_buffer.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("base_frame").value),
            lookup_time,
            # Do not block the single-threaded executor: while this callback
            # waits it cannot receive the TF it is waiting for.  A bounded
            # pending slot below retries after TF has caught up instead.
            timeout=Duration(seconds=0.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sensor_transform = self._tf_buffer.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("sensor_frame").value),
            lookup_time,
            timeout=Duration(seconds=0.0),
        )
        sensor_translation = sensor_transform.transform.translation
        return (
            np.asarray(
                [translation.x, translation.y, translation.z],
                dtype=np.float64,
            ),
            np.asarray(
                [rotation.x, rotation.y, rotation.z, rotation.w],
                dtype=np.float64,
            ),
            np.asarray(
                [
                    sensor_translation.x,
                    sensor_translation.y,
                    sensor_translation.z,
                ],
                dtype=np.float64,
            ),
        )

    def _reject_input(self, count: int, reason: str) -> None:
        if self._layer is not None:
            self._layer.record_rejected_frame(count, reason)
        self._publish_status()

    def _on_observation(self, message: PointCloud2) -> None:
        if self._pending_observation is not None:
            self._dropped_pending_observation_count += 1
        self._pending_observation = (message, time.monotonic())

    def _sample_localization_stability(self) -> None:
        if self._localization_gate.ready:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(seconds=0.0),
            )
            stamp_ns = _stamp_nanoseconds(transform.header.stamp)
            if stamp_ns <= 0 or stamp_ns == self._last_localization_sample_stamp_ns:
                return
            age_sec = abs(
                self.get_clock().now().nanoseconds - stamp_ns
            ) / 1_000_000_000
            if age_sec > float(
                self.get_parameter("maximum_observation_age_sec").value
            ):
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._localization_gate.observe(
                np.asarray(
                    [translation.x, translation.y, translation.z],
                    dtype=np.float64,
                ),
                np.asarray(
                    [rotation.x, rotation.y, rotation.z, rotation.w],
                    dtype=np.float64,
                ),
            )
            self._last_localization_sample_stamp_ns = stamp_ns
        except Exception:
            # AMCL legitimately has no map transform before initial pose.
            return

    def _process_pending_observation(self) -> None:
        self._sample_localization_stability()
        pending = self._pending_observation
        if pending is None:
            return
        message, received_monotonic = pending
        if self._layer is None:
            if (
                time.monotonic() - received_monotonic
                >= float(
                    self.get_parameter("observation_tf_wait_sec").value
                )
            ):
                self._pending_observation = None
                self._reject_input(
                    int(message.width) * int(message.height),
                    "fixed_map_unavailable",
                )
            return
        expected_frame = str(self.get_parameter("map_frame").value)
        if message.header.frame_id != expected_frame:
            self.get_logger().warning(
                "Frozen thermal observation rejected: frame_id "
                f"{message.header.frame_id!r} != {expected_frame!r}",
                throttle_duration_sec=5.0,
            )
            self._pending_observation = None
            self._reject_input(
                int(message.width) * int(message.height),
                "input_frame_mismatch",
            )
            return
        try:
            pose = self._lookup_observation_context(message)
        except Exception as exc:
            elapsed = time.monotonic() - received_monotonic
            wait_limit = max(
                0.0,
                float(
                    self.get_parameter("observation_tf_wait_sec").value
                ),
            )
            if elapsed < wait_limit:
                return
            self.get_logger().warning(
                "Frozen thermal observation has no timestamped map pose: "
                f"{exc}",
                throttle_duration_sec=5.0,
            )
            self._pending_observation = None
            self._reject_input(
                int(message.width) * int(message.height),
                "timestamped_pose_unavailable",
            )
            return
        self._pending_observation = None
        observation_age_sec = abs(
            self.get_clock().now().nanoseconds
            - _stamp_nanoseconds(message.header.stamp)
        ) / 1_000_000_000
        if observation_age_sec > float(
            self.get_parameter("maximum_observation_age_sec").value
        ):
            self._reject_input(
                int(message.width) * int(message.height),
                "stale_observation_timestamp",
            )
            return
        if not self._localization_gate.ready:
            self._layer.last_reason = "localization_stabilizing"
            self._publish_status()
            return
        motion_keyframe = is_keyframe_pose(
            self._last_keyframe_pose,
            pose[0],
            pose[1],
            minimum_translation_m=float(
                self.get_parameter("keyframe_translation_m").value
            ),
            minimum_rotation_rad=math.radians(
                float(self.get_parameter("keyframe_rotation_deg").value)
            ),
        )
        now_monotonic = time.monotonic()
        seconds_since_rejection = (
            now_monotonic - self._last_rejected_integration_monotonic
        )
        if not thermal_update_due(
            motion_keyframe=motion_keyframe,
            seconds_since_success=(
                now_monotonic - self._last_successful_integration_monotonic
            ),
            seconds_since_rejection=seconds_since_rejection,
            stationary_refresh_interval_sec=float(
                self.get_parameter("stationary_refresh_interval_sec").value
            ),
            rejected_frame_retry_sec=float(
                self.get_parameter("rejected_frame_retry_sec").value
            ),
        ):
            self._layer.last_reason = (
                "rejected_frame_backoff"
                if seconds_since_rejection
                < float(
                    self.get_parameter("rejected_frame_retry_sec").value
                )
                else "stationary_frame_skipped"
            )
            self._publish_status()
            return
        try:
            points, temperatures, confidence = self._sample_observations(message)
            result = self._layer.integrate(
                points,
                temperatures,
                confidence,
                observed_at_ns=_stamp_nanoseconds(message.header.stamp),
                association_radius_m=float(
                    self.get_parameter("association_radius_m").value
                ),
                minimum_match_ratio=float(
                    self.get_parameter("minimum_match_ratio").value
                ),
                minimum_observations=int(
                    self.get_parameter("minimum_observations_per_frame").value
                ),
                ema_alpha=float(
                    self.get_parameter("temperature_ema_alpha").value
                ),
                enable_local_alignment=bool(
                    self.get_parameter("enable_local_alignment").value
                ),
                alignment_search_radius_m=float(
                    self.get_parameter("alignment_search_radius_m").value
                ),
                maximum_alignment_translation_m=float(
                    self.get_parameter(
                        "maximum_alignment_translation_m"
                    ).value
                ),
                sensor_origin=pose[2],
                maximum_surface_range_residual_m=float(
                    self.get_parameter(
                        "maximum_surface_range_residual_m"
                    ).value
                ),
            )
        except Exception as exc:
            self._last_rejected_integration_monotonic = now_monotonic
            self.get_logger().warning(
                f"Frozen thermal observation rejected: {exc}",
                throttle_duration_sec=5.0,
            )
            self._reject_input(
                int(message.width) * int(message.height),
                "invalid_observation_cloud",
            )
            return
        if result.accepted:
            self._last_keyframe_pose = (pose[0], pose[1])
            self._last_successful_integration_monotonic = now_monotonic
            self._last_rejected_integration_monotonic = -math.inf
            self._last_observation_at_ns = _stamp_nanoseconds(
                message.header.stamp
            )
        else:
            self._last_rejected_integration_monotonic = now_monotonic
        self._publish_status()

    def _publish_snapshot(self) -> None:
        layer = self._layer
        if layer is None:
            self._publish_status()
            return
        all_observed_count = layer.observed_voxel_count
        # The subset is chosen once from canonical geometry.  Its membership
        # never changes as new areas are observed, so WebUI points do not
        # churn merely because the transport cap was reached.
        observed = self._publish_geometry_indices[
            layer.observation_count[self._publish_geometry_indices] > 0
        ]
        self._snapshot_truncated = observed.shape[0] < all_observed_count
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("map_frame").value)
        message = create_frozen_thermal_cloud(
            header,
            layer.geometry.points[observed],
            layer.temperature_c[observed],
            layer.confidence[observed],
            color_min_c=float(self.get_parameter("color_min_c").value),
            color_max_c=float(self.get_parameter("color_max_c").value),
        )
        self._cloud_publisher.publish(message)
        self._last_published_voxel_count = int(observed.shape[0])
        self._publish_status()

    def persist_if_dirty(self) -> None:
        layer = self._layer
        state_path = self._state_path
        if (
            layer is None
            or not layer.dirty
            or state_path is None
            or not self._persistence_allowed
        ):
            return
        try:
            layer.save_atomic(state_path)
            self._state_error = ""
        except Exception as exc:
            self._state_error = str(exc)
            self.get_logger().error(f"Failed to persist thermal layer: {exc}")
        self._publish_status()

    def _publish_status(self) -> None:
        layer = self._layer
        state_path = self._state_path
        status = {
            "schema_version": 1,
            "session_id": str(self.get_parameter("session_id").value),
            "cumulative": True,
            "fixed_map_available": layer is not None,
            "frame_id": str(self.get_parameter("map_frame").value),
            "geometry_voxel_count": (
                int(layer.geometry.points.shape[0]) if layer is not None else 0
            ),
            "observed_voxel_count": (
                layer.observed_voxel_count if layer is not None else 0
            ),
            "published_voxel_count": self._last_published_voxel_count,
            "snapshot_truncated": self._snapshot_truncated,
            "match_ratio": (
                float(layer.last_match_ratio) if layer is not None else 0.0
            ),
            "rejected_observation_count": (
                int(layer.rejected_observation_count)
                if layer is not None
                else 0
            ),
            "surface_range_rejected_count": (
                int(layer.surface_range_rejected_count)
                if layer is not None
                else 0
            ),
            "accepted_frame_count": (
                int(layer.accepted_frame_count) if layer is not None else 0
            ),
            "rejected_frame_count": (
                int(layer.rejected_frame_count) if layer is not None else 0
            ),
            "dropped_pending_observation_count": (
                self._dropped_pending_observation_count
            ),
            "last_result": (
                layer.last_reason if layer is not None else "fixed_map_unavailable"
            ),
            "last_observation_at": _iso_timestamp(
                self._last_observation_at_ns
            ),
            "persisted_at": _iso_timestamp(
                layer.persisted_at_ns if layer is not None else 0
            ),
            "state_path": str(state_path) if state_path is not None else "",
            "fingerprint": (
                layer.geometry.fingerprint if layer is not None else ""
            ),
            "state_restored": bool(layer.restored) if layer is not None else False,
            "persistence_enabled": bool(
                state_path is not None and self._persistence_allowed
            ),
            "map_error": self._map_error,
            "state_error": self._state_error,
            "local_alignment_enabled": bool(
                self.get_parameter("enable_local_alignment").value
            ),
            "localization_ready": self._localization_gate.ready,
            "localization_stable_sample_count": (
                self._localization_gate.sample_count
            ),
        }
        message = String()
        message.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self._status_publisher.publish(message)

    def close(self) -> None:
        self.persist_if_dirty()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrozenThermalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
