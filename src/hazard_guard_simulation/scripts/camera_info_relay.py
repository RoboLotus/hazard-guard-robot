#!/usr/bin/env python3
"""Republish one-shot camera calibration so late consumers can synchronize."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


CAMERA_INFO_INPUT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CAMERA_INFO_OUTPUT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CameraInfoRelay(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_camera_info_relay")
        self._latest: CameraInfo | None = None
        self.create_subscription(
            CameraInfo,
            "input",
            self._on_info,
            CAMERA_INFO_INPUT_QOS,
        )
        self._publisher = self.create_publisher(
            CameraInfo,
            "output",
            CAMERA_INFO_OUTPUT_QOS,
        )
        self.create_timer(1.0, self._publish_latest)

    def _on_info(self, message: CameraInfo) -> None:
        self._latest = deepcopy(message)
        self._publish_latest()

    def _publish_latest(self) -> None:
        if self._latest is not None:
            self._publisher.publish(self._latest)


def main() -> None:
    rclpy.init()
    node = CameraInfoRelay()
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
