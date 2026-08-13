#!/usr/bin/env python3
"""Turn the raw thermal stream into a blue-to-red image a person can read.

The camera publishes mono16 carrying temperature (Kelvin x 100), not
brightness. Viewers show it as near-black, and the auto-scaling they offer
rescales every frame on its own min and max - so the same 55 C motor changes
shade as the robot turns, and nothing can be compared between frames.

This maps a *fixed* temperature window onto a colour map instead: blue at
min_temp_c, red at max_temp_c, whatever else is in view. Anything outside the
window is clamped, so a hot spot stays the same colour from across the room.

    ros2 run hazard_guard_simulation thermal_colorize.py \
        --ros-args -p min_temp_c:=10.0 -p max_temp_c:=60.0
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Gazebo's thermal camera encodes Kelvin at 0.01 K per count.
KELVIN_PER_COUNT = 0.01
ABSOLUTE_ZERO_C = -273.15


class ThermalColorize(Node):
    def __init__(self) -> None:
        super().__init__("thermal_colorize")
        # The window, not the frame's own range: a fixed mapping is what makes
        # two frames comparable. Defaults span the demo world's floor tiles
        # (10-20 C) up to its hottest equipment (60 C).
        self.declare_parameter("min_temp_c", 10.0)
        self.declare_parameter("max_temp_c", 60.0)
        self.declare_parameter("input_topic", "/thermal_camera/image_raw")
        self.declare_parameter("output_topic", "/thermal_camera/image_color")

        self.bridge = CvBridge()
        # Default (reliable) QoS on both sides: the gz bridge publishes the raw
        # image that way, and rqt_image_view subscribes that way. A best-effort
        # publisher here is silently dropped by the viewer.
        self.publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("input_topic").value, self.on_image, 10
        )
        self.get_logger().info(
            f"{self.get_parameter('min_temp_c').value:.1f} C (파랑) ~ "
            f"{self.get_parameter('max_temp_c').value:.1f} C (빨강) -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def on_image(self, message: Image) -> None:
        low = self.get_parameter("min_temp_c").value
        high = self.get_parameter("max_temp_c").value
        span = max(high - low, 1e-3)

        raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono16")
        celsius = raw.astype(np.float32) * KELVIN_PER_COUNT + ABSOLUTE_ZERO_C
        scaled = np.clip((celsius - low) / span, 0.0, 1.0)
        # JET runs blue -> cyan -> yellow -> red, which is the reading order
        # people expect from a thermal image.
        coloured = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_JET)

        out = self.bridge.cv2_to_imgmsg(coloured, encoding="bgr8")
        out.header = message.header
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = ThermalColorize()
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
