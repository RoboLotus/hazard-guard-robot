from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .cloud import create_thermal_cloud, decode_scalar_image
from .projection import (
    CameraIntrinsics,
    RigidTransform,
    fuse_depth_and_thermal,
)


def _seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _intrinsics(message: CameraInfo) -> CameraIntrinsics:
    fx = float(message.k[0])
    fy = float(message.k[4])
    cx = float(message.k[2])
    cy = float(message.k[5])
    if fx <= 0.0 or fy <= 0.0:
        fx = float(message.p[0])
        fy = float(message.p[5])
        cx = float(message.p[2])
        cy = float(message.p[6])
    return CameraIntrinsics(
        width=int(message.width),
        height=int(message.height),
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
    )


class ThermalDepthFusion(Node):
    """Publish the shared XYZ-temperature-confidence PointCloud2 contract."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_thermal_depth_fusion")
        self.declare_parameter("thermal_scale", 0.01)
        self.declare_parameter("thermal_offset_c", -273.15)
        self.declare_parameter("depth_16u_scale_m", 0.001)
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("sync_tolerance_sec", 0.2)
        self.declare_parameter("output_rate_hz", 2.0)
        self.declare_parameter("depth_horizontal_fov_deg", 73.8)
        self.declare_parameter("thermal_horizontal_fov_deg", 57.0)

        self._depth_image: Image | None = None
        self._depth_info: CameraInfo | None = None
        self._thermal_info: CameraInfo | None = None
        self._last_publish_seconds = -math.inf
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            "/hazard_guard/thermal/points",
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/hazard_guard/thermal/image",
            self._on_thermal,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/hazard_guard/thermal/camera_info",
            self._on_thermal_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/hazard_guard/depth/image",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/hazard_guard/depth/camera_info",
            self._on_depth_info,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Thermal-depth fusion ready; output contract fields are "
            "x, y, z, temperature_c, confidence, pixel_u, pixel_v"
        )

    def _on_depth(self, message: Image) -> None:
        self._depth_image = message

    def _on_depth_info(self, message: CameraInfo) -> None:
        self._depth_info = message

    def _on_thermal_info(self, message: CameraInfo) -> None:
        self._thermal_info = message

    def _on_thermal(self, thermal_image: Image) -> None:
        if self._depth_image is None:
            return
        now_seconds = _seconds(thermal_image.header.stamp)
        output_rate = max(
            0.1, float(self.get_parameter("output_rate_hz").value)
        )
        if now_seconds - self._last_publish_seconds < 1.0 / output_rate:
            return
        if abs(now_seconds - _seconds(self._depth_image.header.stamp)) > float(
            self.get_parameter("sync_tolerance_sec").value
        ):
            self.get_logger().warning(
                "Thermal/depth timestamp difference exceeded tolerance",
                throttle_duration_sec=5.0,
            )
            return

        thermal_frame = thermal_image.header.frame_id
        if not thermal_frame and self._thermal_info is not None:
            thermal_frame = self._thermal_info.header.frame_id
        depth_frame = self._depth_image.header.frame_id
        if not depth_frame and self._depth_info is not None:
            depth_frame = self._depth_info.header.frame_id
        if not thermal_frame or not depth_frame:
            self.get_logger().warning(
                "Thermal/depth frames are missing",
                throttle_duration_sec=5.0,
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                thermal_frame,
                depth_frame,
                Time.from_msg(thermal_image.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            thermal_from_depth = RigidTransform(
                tx=translation.x,
                ty=translation.y,
                tz=translation.z,
                qx=rotation.x,
                qy=rotation.y,
                qz=rotation.z,
                qw=rotation.w,
            )
            depth_scale = (
                float(self.get_parameter("depth_16u_scale_m").value)
                if self._depth_image.encoding in ("mono16", "16UC1", "16SC1")
                else 1.0
            )
            depth = decode_scalar_image(
                self._depth_image,
                scale=depth_scale,
            )
            temperatures = decode_scalar_image(
                thermal_image,
                scale=float(self.get_parameter("thermal_scale").value),
                offset=float(self.get_parameter("thermal_offset_c").value),
            )
            depth_camera = (
                _intrinsics(self._depth_info)
                if self._depth_info is not None
                else CameraIntrinsics.from_horizontal_fov(
                    int(self._depth_image.width),
                    int(self._depth_image.height),
                    float(
                        self.get_parameter("depth_horizontal_fov_deg").value
                    ),
                )
            )
            thermal_camera = (
                _intrinsics(self._thermal_info)
                if self._thermal_info is not None
                else CameraIntrinsics.from_horizontal_fov(
                    int(thermal_image.width),
                    int(thermal_image.height),
                    float(
                        self.get_parameter("thermal_horizontal_fov_deg").value
                    ),
                )
            )
            points = fuse_depth_and_thermal(
                depth,
                depth_camera,
                temperatures,
                thermal_camera,
                thermal_from_depth,
                stride=int(self.get_parameter("stride").value),
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Thermal-depth fusion skipped: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        header = Header()
        header.stamp = thermal_image.header.stamp
        header.frame_id = thermal_frame
        self._publisher.publish(create_thermal_cloud(header, points))
        self._last_publish_seconds = now_seconds


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThermalDepthFusion()
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
