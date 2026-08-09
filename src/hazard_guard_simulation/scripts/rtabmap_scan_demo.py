#!/usr/bin/env python3
"""Bounded simulation-only scan route for building an RTAB-Map demo cloud."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RtabmapScanDemo(Node):
    """Scan the demo facility's south aisle without turning at narrow ends."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_rtabmap_scan_demo")
        if not self.get_parameter("use_sim_time").value:
            raise RuntimeError(
                "This route is simulation-only and requires use_sim_time:=true."
            )

        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._segments = [
            ("rotate", math.tau, 0.32, "initial 360-degree color scan"),
            ("drive", 1.80, 0.14, "scan east to the safe interior endpoint"),
            ("drive", 1.80, -0.14, "return west to P2 without turning"),
            ("drive", 1.00, -0.14, "scan west to the safe interior endpoint"),
            ("drive", 1.00, 0.14, "return east to P2 without turning"),
        ]
        self._segment_index = 0
        self._pose: tuple[float, float, float] | None = None
        self._segment_start: tuple[float, float, float] | None = None
        self._last_yaw: float | None = None
        self._accumulated_yaw = 0.0
        self._pause_until = 0.0
        self._started_at_ns: int | None = None
        self.finished = False
        self.create_timer(0.05, self._update)
        self.get_logger().info(
            "Simulation-only RTAB-Map scan route is waiting for odometry."
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        if self._last_yaw is not None:
            self._accumulated_yaw += normalize_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        self._pose = (float(position.x), float(position.y), yaw)

    def _stop(self) -> None:
        self._publisher.publish(Twist())

    def _begin_segment(self) -> None:
        self._segment_start = self._pose
        self._accumulated_yaw = 0.0
        mode, target, _, label = self._segments[self._segment_index]
        unit = "rad" if mode == "rotate" else "m"
        self.get_logger().info(
            f"Segment {self._segment_index + 1}/{len(self._segments)}: "
            f"{label} ({target:.2f} {unit})"
        )

    def _advance(self) -> None:
        self._stop()
        self._segment_index += 1
        self._segment_start = None
        self._pause_until = time.monotonic() + 0.8
        if self._segment_index >= len(self._segments):
            self.finished = True
            self.get_logger().info("RTAB-Map scan route completed safely.")

    def _update(self) -> None:
        if self.finished:
            self._stop()
            return
        simulation_now = self.get_clock().now().nanoseconds
        if simulation_now > 0 and self._started_at_ns is None:
            self._started_at_ns = simulation_now
        if (
            self._started_at_ns is not None
            and simulation_now - self._started_at_ns > 150_000_000_000
        ):
            self._stop()
            self.finished = True
            self.get_logger().error(
                "Scan route exceeded 150 simulated seconds; "
                "the robot was stopped."
            )
            return
        if simulation_now == 0 or self._pose is None:
            return
        if time.monotonic() < self._pause_until:
            self._stop()
            return
        if self._segment_start is None:
            self._begin_segment()

        mode, target, speed, _ = self._segments[self._segment_index]
        command = Twist()
        if mode == "rotate":
            if abs(self._accumulated_yaw) >= abs(target) * 0.985:
                self._advance()
                return
            command.angular.z = speed
        else:
            distance = math.hypot(
                self._pose[0] - self._segment_start[0],
                self._pose[1] - self._segment_start[1],
            )
            if distance >= target * 0.985:
                self._advance()
                return
            command.linear.x = speed
        self._publisher.publish(command)


def main() -> None:
    rclpy.init()
    node = RtabmapScanDemo()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(3):
            node._stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
