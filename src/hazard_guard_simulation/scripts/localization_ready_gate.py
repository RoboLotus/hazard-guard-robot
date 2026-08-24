#!/usr/bin/env python3
"""Gate second-pass RGB-D capture on a valid, active 2D localization stack."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def validate_map_files(map_path: str) -> tuple[Path, Path]:
    """Return resolved map YAML/image paths or raise a useful error."""

    yaml_path = Path(map_path).expanduser().resolve()
    if not yaml_path.is_file():
        raise ValueError(f"map YAML does not exist: {yaml_path}")

    try:
        contents = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read map YAML {yaml_path}: {exc}") from exc
    if not isinstance(contents, dict):
        raise ValueError(f"map YAML must contain a mapping: {yaml_path}")

    image_value = contents.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ValueError(f"map YAML has no valid image entry: {yaml_path}")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise ValueError(f"map image does not exist or is empty: {image_path}")
    return yaml_path, image_path


def transform_is_fresh(
    *,
    now_ns: int,
    stamp_ns: int,
    max_age_sec: float,
    future_tolerance_sec: float,
) -> bool:
    """Check that a dynamic transform is recent in the active ROS clock."""

    if now_ns <= 0 or stamp_ns <= 0:
        return False
    age_sec = (now_ns - stamp_ns) / 1e9
    return -future_tolerance_sec <= age_sec <= max_age_sec


class LocalizationReadyGate(Node):
    """Exit successfully only after map localization is safe for 3D capture."""

    def __init__(self) -> None:
        super().__init__("localization_ready_gate")
        self.declare_parameter("map_path", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("stable_samples", 3)
        self.declare_parameter("max_tf_age_sec", 2.0)
        self.declare_parameter("future_tolerance_sec", 0.5)

        self.result_code: int | None = None
        self._started_at = time.monotonic()
        self._stable_count = 0
        self._last_status = ""
        self._map_path = str(self.get_parameter("map_path").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._stable_samples = int(self.get_parameter("stable_samples").value)
        self._max_tf_age_sec = float(
            self.get_parameter("max_tf_age_sec").value
        )
        self._future_tolerance_sec = float(
            self.get_parameter("future_tolerance_sec").value
        )

        if self._timeout_sec <= 0.0:
            self._fail("timeout_sec must be positive", 2)
            return
        if self._stable_samples < 1:
            self._fail("stable_samples must be at least 1", 2)
            return
        valid_tf_age = (
            math.isfinite(self._max_tf_age_sec)
            and self._max_tf_age_sec > 0
        )
        if not valid_tf_age:
            self._fail("max_tf_age_sec must be finite and positive", 2)
            return

        try:
            yaml_path, image_path = validate_map_files(self._map_path)
        except ValueError as exc:
            self._fail(str(exc), 2)
            return
        self.get_logger().info(
            f"Validated localization map: {yaml_path} (image: {image_path})"
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lifecycle_clients = {
            "map_server": self.create_client(
                GetState, "/map_server/get_state"
            ),
            "amcl": self.create_client(GetState, "/amcl/get_state"),
        }
        self._lifecycle_futures: dict[str, object | None] = {
            name: None for name in self._lifecycle_clients
        }
        self._lifecycle_active = {
            name: False for name in self._lifecycle_clients
        }
        self._timer = self.create_timer(0.2, self._poll)

    def _fail(self, message: str, code: int) -> None:
        self.get_logger().error(message)
        self.result_code = code

    def _request_lifecycle_states(self) -> None:
        for name, client in self._lifecycle_clients.items():
            future = self._lifecycle_futures[name]
            if future is not None:
                if future.done():
                    try:
                        response = future.result()
                        self._lifecycle_active[name] = (
                            response.current_state.id
                            == State.PRIMARY_STATE_ACTIVE
                        )
                    except Exception as exc:  # ROS future error boundary
                        self.get_logger().debug(
                            f"{name} lifecycle query failed: {exc}"
                        )
                        self._lifecycle_active[name] = False
                    self._lifecycle_futures[name] = None
                continue
            if client.service_is_ready():
                self._lifecycle_futures[name] = client.call_async(
                    GetState.Request()
                )

    def _has_fresh_transform(self) -> bool:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(),
            )
        except TransformException:
            return False
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        return transform_is_fresh(
            now_ns=self.get_clock().now().nanoseconds,
            stamp_ns=stamp_ns,
            max_age_sec=self._max_tf_age_sec,
            future_tolerance_sec=self._future_tolerance_sec,
        )

    def _poll(self) -> None:
        if self.result_code is not None:
            return
        if time.monotonic() - self._started_at > self._timeout_sec:
            self._fail(
                "Localization readiness timed out; RTAB-Map will not start",
                3,
            )
            return

        self._request_lifecycle_states()
        lifecycle_ready = all(self._lifecycle_active.values())
        tf_ready = lifecycle_ready and self._has_fresh_transform()
        self._stable_count = self._stable_count + 1 if tf_ready else 0

        map_state = (
            "active" if self._lifecycle_active["map_server"] else "waiting"
        )
        amcl_state = (
            "active" if self._lifecycle_active["amcl"] else "waiting"
        )
        status = (
            f"map_server={map_state}, amcl={amcl_state}, "
            f"fresh_tf={tf_ready}, stable={self._stable_count}/"
            f"{self._stable_samples}"
        )
        if status != self._last_status:
            self.get_logger().info(f"Waiting for localization: {status}")
            self._last_status = status

        if self._stable_count >= self._stable_samples:
            self.get_logger().info(
                "Localization is active and stable; allowing RTAB-Map capture"
            )
            self.result_code = 0


def main() -> None:
    rclpy.init()
    node = LocalizationReadyGate()
    try:
        while rclpy.ok() and node.result_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        result_code = node.result_code if node.result_code is not None else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(result_code)


if __name__ == "__main__":
    main()
