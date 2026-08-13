#!/usr/bin/env python3
"""Draw where the depth camera is looking onto the thermal image.

A calibration is a claim about geometry, and this is the cheapest way to see
whether the claim holds: take the point the depth camera has centred, put it
through the extrinsic, and mark it on the thermal frame. If the transform is
right the marker lands on the same physical thing the depth camera centred -
the same motor edge, the same corner. If it is wrong the marker sits beside it,
and the gap is the error in pixels.

The depth value is what makes this exact. A pixel on its own is a ray, and the
two cameras are 68 mm apart, so the same ray lands 20 px away at 0.5 m and 3 px
away at 3 m. With depth the point is fixed in space and there is nothing to
guess.

Nothing here is simulation-specific: intrinsics come from camera_info and the
extrinsic from TF, so the same node runs against the real robot once its
calibration publishes those.

    ros2 run hazard_guard_simulation thermal_overlay.py
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

MARKER = (255, 255, 255)
MISS = (0, 0, 255)


def rotation_from_quaternion(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ThermalOverlay(Node):
    def __init__(self) -> None:
        super().__init__("thermal_overlay")
        self.declare_parameter("depth_image", "/depth_camera/image_raw")
        self.declare_parameter("depth_info", "/depth_camera/camera_info")
        self.declare_parameter("thermal_image", "/thermal_camera/image_color")
        self.declare_parameter("thermal_info", "/thermal_camera/camera_info")
        self.declare_parameter("output_topic", "/thermal_camera/image_overlay")
        # Depth and RGB share one optical frame on this robot, so the depth
        # centre ray is the RGB centre ray; one marker covers both.
        self.declare_parameter("depth_frame", "depth_camera_optical_frame")
        self.declare_parameter("thermal_frame", "thermal_camera_optical_frame")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_info: CameraInfo | None = None
        self.thermal_info: CameraInfo | None = None

        self.create_subscription(
            CameraInfo, self.get_parameter("depth_info").value,
            lambda m: setattr(self, "depth_info", m), 10,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("thermal_info").value,
            lambda m: setattr(self, "thermal_info", m), 10,
        )
        self.publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").value, 10
        )

        # The two cameras run at different rates (10 Hz and 8.7 Hz) and never
        # line up exactly, so pair them by closest stamp rather than dropping
        # everything that does not match.
        synchronizer = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, self.get_parameter("depth_image").value),
                Subscriber(self, Image, self.get_parameter("thermal_image").value),
            ],
            queue_size=10,
            slop=0.15,
        )
        synchronizer.registerCallback(self.on_pair)
        self.get_logger().info(
            f"{self.get_parameter('depth_frame').value} 중심점 -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def centre_point(self, depth_message: Image):
        """3D point the depth camera has centred, in its own optical frame."""
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        v, u = depth.shape[0] // 2, depth.shape[1] // 2
        # One pixel is noisy and can be a hole; the median of a small patch is
        # the depth of whatever surface fills the centre of the frame.
        patch = depth[v - 3:v + 4, u - 3:u + 4].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None, None
        z = float(np.median(valid))
        k = self.depth_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]), z

    def on_pair(self, depth_message: Image, thermal_message: Image) -> None:
        if self.depth_info is None or self.thermal_info is None:
            return
        frame = self.bridge.imgmsg_to_cv2(thermal_message, desired_encoding="bgr8").copy()
        height, width = frame.shape[:2]

        # The thermal centre, for reference: the gap between the two markers is
        # the parallax, and it has to shrink as the target gets further away.
        centre = (width // 2, height // 2)
        cv2.drawMarker(frame, centre, (140, 140, 140), cv2.MARKER_CROSS, 9, 1)

        point, distance = self.centre_point(depth_message)
        if point is None:
            self.draw_note(frame, "no depth at centre", MISS)
            self.publish(frame, thermal_message)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("thermal_frame").value,
                self.get_parameter("depth_frame").value,
                rclpy.time.Time(),
            )
        except Exception as error:  # noqa: BLE001 - any TF failure is the same here
            self.draw_note(frame, "no TF", MISS)
            self.get_logger().warn(f"TF lookup failed: {error}", throttle_duration_sec=5.0)
            self.publish(frame, thermal_message)
            return

        translation = transform.transform.translation
        in_thermal = rotation_from_quaternion(transform.transform.rotation) @ point + np.array(
            [translation.x, translation.y, translation.z]
        )
        if in_thermal[2] <= 0:
            self.draw_note(frame, "behind camera", MISS)
            self.publish(frame, thermal_message)
            return

        k = self.thermal_info.k
        u = int(round(k[0] * in_thermal[0] / in_thermal[2] + k[2]))
        v = int(round(k[4] * in_thermal[1] / in_thermal[2] + k[5]))

        inside = 0 <= u < width and 0 <= v < height
        colour = MARKER if inside else MISS
        if inside:
            cv2.line(frame, centre, (u, v), colour, 1)
            cv2.drawMarker(frame, (u, v), colour, cv2.MARKER_TILTED_CROSS, 11, 1)
            cv2.circle(frame, (u, v), 6, colour, 1)
        # The gap to the thermal centre is the parallax, and it has to shrink
        # with distance: 68 mm of baseline is 10 px at 1 m and 3 px at 3 m.
        offset = f"{u - width // 2:+d},{v - height // 2:+d}"
        self.draw_note(
            frame,
            f"d={distance:.2f}m  off={offset}px" + ("" if inside else " OUT"),
            colour,
        )
        self.publish(frame, thermal_message)

    @staticmethod
    def draw_note(frame, text, colour) -> None:
        # Thermal frames are 160 px wide; a fixed font size covers the picture.
        scale = frame.shape[1] / 640.0 * 1.1
        cv2.putText(frame, text, (3, frame.shape[0] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)

    def publish(self, frame, source: Image) -> None:
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header = source.header
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = ThermalOverlay()
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
