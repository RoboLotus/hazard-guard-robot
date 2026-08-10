#!/usr/bin/env python3
"""Republish one-shot camera calibration so late consumers can synchronize."""
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__("hazard_guard_camera_info_relay")
        self._latest = None
        self.create_subscription(CameraInfo, "input", self._on_info, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(CameraInfo, "output", qos)
        self.create_timer(1.0, self._publish_latest)

    def _on_info(self, message):
        self._latest = deepcopy(message)
        self._publish_latest()

    def _publish_latest(self):
        if self._latest is not None:
            self._publisher.publish(self._latest)


def main():
    rclpy.init()
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
