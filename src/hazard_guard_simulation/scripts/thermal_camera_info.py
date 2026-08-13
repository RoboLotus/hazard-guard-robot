#!/usr/bin/env python3
"""Publish the thermal camera's intrinsics, because Gazebo's are wrong.

Fortress' ThermalCameraSensor fills CameraInfo from the default camera rather
than the one in the SDF. Measured against a 160 x 120 / 57 deg sensor it
publishes fx 277.0 and centre (160, 120) - the numbers for a 320 x 240 / 60 deg
camera, which is Gazebo's default. The RGB and depth sensors are unaffected and
report correctly (fx 426.2, centre 320 x 240).

Left alone this quietly ruins anything geometric: projecting the depth centre
with those numbers puts it 85 px off, outside a 160 px frame.

So the intrinsics come from the sensor profile instead, stamped to match each
image. That is also how the real robot will work - the TMC160B is a USB video
device with no driver-side calibration, so its CameraInfo has to be published
from a calibration file. Same node, different source of numbers.

    ros2 run hazard_guard_simulation thermal_camera_info.py
"""
from __future__ import annotations

import math

import rclpy
from hazard_guard_sensor_config import TMC160B
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ThermalCameraInfo(Node):
    def __init__(self) -> None:
        super().__init__("thermal_camera_info")
        self.declare_parameter("image_topic", "/thermal_camera/image_raw")
        self.declare_parameter("info_topic", "/thermal_camera/camera_info")
        self.declare_parameter("horizontal_fov_deg", TMC160B.horizontal_fov_deg)
        # Zero distortion is the simulator's truth, not the sensor's. On real
        # hardware these five numbers come from the calibration run.
        self.declare_parameter("distortion", [0.0, 0.0, 0.0, 0.0, 0.0])

        self.publisher = self.create_publisher(
            CameraInfo, self.get_parameter("info_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 10
        )
        self.template: CameraInfo | None = None

    def build(self, width: int, height: int, frame_id: str) -> CameraInfo:
        fov = math.radians(self.get_parameter("horizontal_fov_deg").value)
        # Square pixels: the vertical field follows from the aspect ratio, so
        # one focal length describes both axes.
        fx = fy = (width / 2) / math.tan(fov / 2)
        cx, cy = width / 2, height / 2

        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = list(self.get_parameter("distortion").value)
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f"{width}x{height}  fov {math.degrees(fov):.1f} deg  "
            f"fx {fx:.1f}  centre ({cx:.0f}, {cy:.0f})"
        )
        return info

    def on_image(self, message: Image) -> None:
        if self.template is None or self.template.width != message.width:
            self.template = self.build(
                message.width, message.height, message.header.frame_id
            )
        self.template.header.stamp = message.header.stamp
        self.publisher.publish(self.template)


def main() -> None:
    rclpy.init()
    node = ThermalCameraInfo()
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
