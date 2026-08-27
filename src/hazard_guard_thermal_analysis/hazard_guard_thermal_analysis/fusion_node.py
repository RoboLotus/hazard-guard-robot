from __future__ import annotations

from collections import deque
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .cloud import create_thermal_cloud, decode_scalar_array
from .projection import (
    CameraIntrinsics,
    RigidTransform,
    ThermalPoint,
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
        distortion=tuple(float(value) for value in message.d),
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
        self.declare_parameter("sync_by_receipt_time", False)
        self.declare_parameter("output_frame", "")
        self.declare_parameter("transform_at_latest", False)
        self.declare_parameter("output_rate_hz", 2.0)
        self.declare_parameter("color_min_c", 10.0)
        self.declare_parameter("color_max_c", 60.0)
        self.declare_parameter("depth_horizontal_fov_deg", 73.8)
        self.declare_parameter("thermal_horizontal_fov_deg", 57.0)
        self.declare_parameter("thermal_sampling_mode", "bilinear")

        self._depth_images: deque[tuple[float, Image]] = deque(maxlen=30)
        self._depth_info: CameraInfo | None = None
        self._thermal_info: CameraInfo | None = None
        self._last_publish_monotonic = -math.inf
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
            "x, y, z, temperature_c, confidence, pixel_u, pixel_v, rgb"
        )

    def _on_depth(self, message: Image) -> None:
        self._depth_images.append((time.monotonic(), message))

    def _on_depth_info(self, message: CameraInfo) -> None:
        self._depth_info = message

    def _on_thermal_info(self, message: CameraInfo) -> None:
        self._thermal_info = message

    def _on_thermal(self, thermal_image: Image) -> None:
        if not self._depth_images:
            return
        now_monotonic = time.monotonic()
        output_rate = max(
            0.1, float(self.get_parameter("output_rate_hz").value)
        )
        if now_monotonic - self._last_publish_monotonic < 1.0 / output_rate:
            return

        receipt_sync = bool(self.get_parameter("sync_by_receipt_time").value)
        if receipt_sync:
            depth_receipt, depth_image = min(
                self._depth_images,
                key=lambda sample: abs(sample[0] - now_monotonic),
            )
            pair_delta = abs(depth_receipt - now_monotonic)
        else:
            thermal_seconds = _seconds(thermal_image.header.stamp)
            depth_receipt, depth_image = min(
                self._depth_images,
                key=lambda sample: abs(
                    _seconds(sample[1].header.stamp) - thermal_seconds
                ),
            )
            pair_delta = abs(
                _seconds(depth_image.header.stamp) - thermal_seconds
            )
        tolerance = float(self.get_parameter("sync_tolerance_sec").value)
        if pair_delta > tolerance:
            self.get_logger().warning(
                "Thermal/depth pairing delta exceeded tolerance: "
                f"{pair_delta:.3f}s > {tolerance:.3f}s "
                f"({'receipt' if receipt_sync else 'header'} time)",
                throttle_duration_sec=5.0,
            )
            return

        thermal_frame = thermal_image.header.frame_id
        if not thermal_frame and self._thermal_info is not None:
            thermal_frame = self._thermal_info.header.frame_id
        depth_frame = depth_image.header.frame_id
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
                Time(),
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
                if depth_image.encoding in ("mono16", "16UC1", "16SC1")
                else 1.0
            )
            depth = decode_scalar_array(
                depth_image,
                scale=depth_scale,
            ).reshape(-1)
            temperatures = decode_scalar_array(
                thermal_image,
                scale=float(self.get_parameter("thermal_scale").value),
                offset=float(self.get_parameter("thermal_offset_c").value),
            ).reshape(-1)
            depth_camera = (
                _intrinsics(self._depth_info)
                if self._depth_info is not None
                else CameraIntrinsics.from_horizontal_fov(
                    int(depth_image.width),
                    int(depth_image.height),
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
                thermal_sampling_mode=str(
                    self.get_parameter("thermal_sampling_mode").value
                ),
            )

            output_frame = str(
                self.get_parameter("output_frame").value
            ).strip()
            output_stamp = thermal_image.header.stamp
            if output_frame and output_frame != thermal_frame:
                lookup_time = (
                    Time()
                    if bool(self.get_parameter("transform_at_latest").value)
                    else Time.from_msg(thermal_image.header.stamp)
                )
                output_transform = self._tf_buffer.lookup_transform(
                    output_frame,
                    thermal_frame,
                    lookup_time,
                    timeout=Duration(seconds=0.15),
                )
                output_from_thermal = RigidTransform(
                    tx=output_transform.transform.translation.x,
                    ty=output_transform.transform.translation.y,
                    tz=output_transform.transform.translation.z,
                    qx=output_transform.transform.rotation.x,
                    qy=output_transform.transform.rotation.y,
                    qz=output_transform.transform.rotation.z,
                    qw=output_transform.transform.rotation.w,
                )
                points = [
                    ThermalPoint(
                        *output_from_thermal.apply(point.x, point.y, point.z),
                        point.temperature_c,
                        point.confidence,
                        point.pixel_u,
                        point.pixel_v,
                    )
                    for point in points
                ]
                if bool(self.get_parameter("transform_at_latest").value):
                    output_stamp = output_transform.header.stamp
                thermal_frame = output_frame
        except Exception as exc:
            self.get_logger().warning(
                f"Thermal-depth fusion skipped: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        header = Header()
        header.stamp = output_stamp
        header.frame_id = thermal_frame
        self._publisher.publish(create_thermal_cloud(
            header,
            points,
            color_min_c=float(self.get_parameter("color_min_c").value),
            color_max_c=float(self.get_parameter("color_max_c").value),
        ))
        self._last_publish_monotonic = now_monotonic


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
