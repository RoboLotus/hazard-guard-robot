#!/usr/bin/env python3
"""Apply a selectable timestamp policy to a visualization point cloud."""

from __future__ import annotations

import math
from typing import Final

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


STAMP_MODES: Final = {"preserve", "offset", "latest"}
NANOSECONDS_PER_SECOND: Final = 1_000_000_000
MAX_ROS_TIME_SECONDS: Final = 2_147_483_647


def adjusted_stamp(stamp: Time, mode: str, offset_sec: float) -> Time:
    """Return a new stamp according to the configured correction policy."""

    if mode not in STAMP_MODES:
        raise ValueError(f"unsupported cloud stamp mode: {mode}")
    if mode == "latest":
        return Time()
    if mode == "preserve":
        return Time(sec=stamp.sec, nanosec=stamp.nanosec)
    if not math.isfinite(offset_sec):
        raise ValueError("stamp_offset_sec must be finite")

    source_ns = stamp.sec * NANOSECONDS_PER_SECOND + stamp.nanosec
    offset_ns = round(offset_sec * NANOSECONDS_PER_SECOND)
    corrected_ns = max(0, source_ns + offset_ns)
    seconds, nanoseconds = divmod(corrected_ns, NANOSECONDS_PER_SECOND)
    if seconds > MAX_ROS_TIME_SECONDS:
        raise ValueError("corrected cloud timestamp exceeds ROS Time range")
    return Time(sec=int(seconds), nanosec=int(nanoseconds))


class CloudStampRelay(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_cloud_stamp_relay")
        self.declare_parameter("stamp_mode", "latest")
        self.declare_parameter("stamp_offset_sec", 0.0)
        self._stamp_mode = str(self.get_parameter("stamp_mode").value)
        self._stamp_offset_sec = float(
            self.get_parameter("stamp_offset_sec").value
        )
        if self._stamp_mode not in STAMP_MODES:
            raise ValueError(
                f"stamp_mode must be one of {sorted(STAMP_MODES)}, "
                f"got {self._stamp_mode!r}"
            )
        if not math.isfinite(self._stamp_offset_sec):
            raise ValueError("stamp_offset_sec must be finite")
        self._publisher = self.create_publisher(
            PointCloud2, "output", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "input", self._on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Cloud timestamp policy: "
            f"mode={self._stamp_mode}, offset={self._stamp_offset_sec:.6f}s"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        cloud = PointCloud2()
        cloud.header.frame_id = message.header.frame_id
        cloud.header.stamp = adjusted_stamp(
            message.header.stamp,
            self._stamp_mode,
            self._stamp_offset_sec,
        )
        cloud.height = message.height
        cloud.width = message.width
        cloud.fields = message.fields
        cloud.is_bigendian = message.is_bigendian
        cloud.point_step = message.point_step
        cloud.row_step = message.row_step
        cloud.data = message.data
        cloud.is_dense = message.is_dense
        self._publisher.publish(cloud)


def main() -> None:
    rclpy.init()
    node = CloudStampRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
