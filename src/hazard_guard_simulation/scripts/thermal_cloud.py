#!/usr/bin/env python3
"""Paint the SLAM map with temperature instead of colour.

The calibration answered where a point in the depth optical frame lands in the
thermal one. thermal_overlay.py spends that answer on a single pixel to prove
it; this node spends it on every pixel to build a map. Each depth frame becomes
a cloud of points, each point is projected into the thermal frame and reads the
temperature that sits there, and the result is carried into the map frame by
the SLAM transform and dropped into a voxel grid. What accumulates is a thermal
3D map of everything the robot has driven past.

The depth value is what makes the projection exact - see thermal_overlay.py for
why. What it also does here is fix scale: a point 4 m away is one voxel, not a
smear, so a hot spot stays in one place while the robot moves around it.

Temperature per voxel is an exponential moving average, not a single reading.
One frame's noise should not repaint a wall, and a motor that cools down should
show it rather than keep the hottest value it ever had.

Nothing here is simulation-specific: intrinsics come from camera_info and the
extrinsic from TF, so the same node runs against the real robot once its
calibration publishes those.

    ros2 run hazard_guard_simulation thermal_cloud.py
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

# Gazebo's thermal camera encodes Kelvin at 0.01 K per count, the same
# convention thermal_colorize.py reads.
KELVIN_PER_COUNT = 0.01
ABSOLUTE_ZERO_C = -273.15

POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])


def rotation_from_quaternion(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_matrix(transform) -> tuple[np.ndarray, np.ndarray]:
    translation = transform.transform.translation
    return (
        rotation_from_quaternion(transform.transform.rotation),
        np.array([translation.x, translation.y, translation.z]),
    )


def back_project(depth: np.ndarray, k, stride: int, near: float, far: float) -> np.ndarray:
    """Depth image -> Nx3 points in the depth optical frame.

    Sampling every `stride`-th pixel is what keeps this real time. A 640x480
    frame at stride 4 is 19 200 rays, and at 5 cm voxels a denser sample only
    lands more points in voxels that are already filled.
    """
    sampled = depth[::stride, ::stride].astype(np.float32)
    rows, columns = np.nonzero(np.isfinite(sampled) & (sampled > near) & (sampled < far))
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    z = sampled[rows, columns]
    u = columns.astype(np.float32) * stride
    v = rows.astype(np.float32) * stride
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))


def project(points: np.ndarray, k) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Points in an optical frame -> pixel coordinates plus an in-front mask."""
    in_front = points[:, 2] > 0
    z = np.where(in_front, points[:, 2], 1.0)
    u = k[0] * points[:, 0] / z + k[2]
    v = k[4] * points[:, 1] / z + k[5]
    return u, v, in_front


def voxel_average(points: np.ndarray, values: np.ndarray, voxel_size: float):
    """Collapse points to voxel indices, averaging the values that land in each.

    Averaging inside the frame first means one reading per voxel per frame, so
    a surface seen edge-on by 200 rays does not outvote one seen by 10.
    """
    keys = np.floor(points / voxel_size).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    totals = np.bincount(inverse, weights=values)
    counts = np.bincount(inverse)
    return unique, totals / counts


def temperature_colors(temperatures: np.ndarray, low: float, high: float) -> np.ndarray:
    """Fixed temperature window -> packed 0x00RRGGBB, blue cold to red hot.

    Fixed, not per-frame: the same 55 C motor has to be the same colour from
    across the room, which is the whole point of mapping it.
    """
    span = max(high - low, 1e-3)
    scaled = np.clip((temperatures - low) / span, 0.0, 1.0)
    ramp = (scaled * 255).astype(np.uint8).reshape(-1, 1)
    bgr = cv2.applyColorMap(ramp, cv2.COLORMAP_JET).reshape(-1, 3).astype(np.uint32)
    return (bgr[:, 2] << 16) | (bgr[:, 1] << 8) | bgr[:, 0]


def cloud_message(points: np.ndarray, colors: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    records = np.empty(points.shape[0], dtype=POINT_DTYPE)
    records["x"] = points[:, 0]
    records["y"] = points[:, 1]
    records["z"] = points[:, 2]
    # The PCL convention: 32 bits of packed colour carried in a float field.
    records["rgb"] = colors.astype(np.uint32).view(np.float32)

    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.height = 1
    message.width = records.shape[0]
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = POINT_DTYPE.itemsize
    message.row_step = POINT_DTYPE.itemsize * records.shape[0]
    message.data = records.tobytes()
    message.is_dense = True
    return message


class ThermalCloud(Node):
    def __init__(self) -> None:
        super().__init__("thermal_cloud")
        self.declare_parameter("depth_image", "/depth_camera/image_raw")
        self.declare_parameter("depth_info", "/depth_camera/camera_info")
        # The raw mono16 stream, not the colourised one: this needs the
        # temperature, and image_color has already thrown it away.
        self.declare_parameter("thermal_image", "/thermal_camera/image_raw")
        self.declare_parameter("thermal_info", "/thermal_camera/camera_info")
        self.declare_parameter("output_topic", "/hazard_guard/thermal_cloud")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("stride", 4)
        # Depth is trustworthy in the middle of its range; the near edge is
        # noise and the far edge is where the 68 mm baseline stops mattering.
        self.declare_parameter("range_min_m", 0.25)
        self.declare_parameter("range_max_m", 5.0)
        self.declare_parameter("min_temp_c", 10.0)
        self.declare_parameter("max_temp_c", 60.0)
        # A wall repainted by one noisy frame is worse than a slow response.
        self.declare_parameter("temperature_alpha", 0.35)
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("max_voxels", 400000)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_info: CameraInfo | None = None
        self.thermal_info: CameraInfo | None = None
        # Voxel index -> temperature in Celsius. The position is the voxel
        # centre, so nothing else has to be stored.
        self.voxels: dict[tuple[int, int, int], float] = {}

        self.create_subscription(
            CameraInfo, self.get_parameter("depth_info").value,
            lambda m: setattr(self, "depth_info", m), 10,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("thermal_info").value,
            lambda m: setattr(self, "thermal_info", m), 10,
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, 1
        )

        # The two cameras run at different rates (10 Hz and 8.7 Hz), the same
        # reason thermal_overlay.py pairs by closest stamp rather than exactly.
        synchronizer = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, self.get_parameter("depth_image").value),
                Subscriber(self, Image, self.get_parameter("thermal_image").value),
            ],
            queue_size=10,
            slop=0.15,
        )
        synchronizer.registerCallback(self.on_pair)
        self.create_timer(
            self.get_parameter("publish_period_sec").value, self.publish_cloud
        )
        self.get_logger().info(
            f"{self.get_parameter('min_temp_c').value:.1f} C ~ "
            f"{self.get_parameter('max_temp_c').value:.1f} C -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def on_pair(self, depth_message: Image, thermal_message: Image) -> None:
        if self.depth_info is None or self.thermal_info is None:
            return

        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        thermal = self.bridge.imgmsg_to_cv2(thermal_message, desired_encoding="passthrough")
        if thermal.ndim != 2:
            self.get_logger().warn(
                "thermal_image must carry temperature (mono16), not a colour "
                "image; point it at the raw stream",
                throttle_duration_sec=10.0,
            )
            return

        points = back_project(
            depth,
            self.depth_info.k,
            int(self.get_parameter("stride").value),
            float(self.get_parameter("range_min_m").value),
            float(self.get_parameter("range_max_m").value),
        )
        if points.shape[0] == 0:
            return

        depth_frame = depth_message.header.frame_id
        try:
            to_thermal = self.tf_buffer.lookup_transform(
                thermal_message.header.frame_id, depth_frame, rclpy.time.Time()
            )
            to_map = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value, depth_frame, rclpy.time.Time()
            )
        except Exception as error:  # noqa: BLE001 - any TF failure is the same here
            self.get_logger().warn(
                f"TF lookup failed: {error}", throttle_duration_sec=5.0
            )
            return

        rotation, translation = transform_matrix(to_thermal)
        u, v, in_front = project(points @ rotation.T + translation, self.thermal_info.k)
        column = np.round(u).astype(np.int64)
        row = np.round(v).astype(np.int64)
        height, width = thermal.shape[:2]
        seen = (
            in_front
            & (column >= 0) & (column < width)
            & (row >= 0) & (row < height)
        )
        if not seen.any():
            return

        celsius = thermal[row[seen], column[seen]].astype(np.float32) * KELVIN_PER_COUNT
        celsius += ABSOLUTE_ZERO_C

        rotation, translation = transform_matrix(to_map)
        in_map = points[seen] @ rotation.T + translation
        keys, temperatures = voxel_average(
            in_map, celsius, float(self.get_parameter("voxel_size").value)
        )
        self.merge(keys, temperatures)

    def merge(self, keys: np.ndarray, temperatures: np.ndarray) -> None:
        alpha = float(self.get_parameter("temperature_alpha").value)
        limit = int(self.get_parameter("max_voxels").value)
        for key, temperature in zip(map(tuple, keys.tolist()), temperatures.tolist()):
            previous = self.voxels.get(key)
            self.voxels[key] = (
                temperature if previous is None
                else previous + alpha * (temperature - previous)
            )
        # ponytail: oldest-first eviction on a plain dict. Fine for one
        # building; swap for an octree if the map has to outgrow memory.
        while len(self.voxels) > limit:
            self.voxels.pop(next(iter(self.voxels)))

    def publish_cloud(self) -> None:
        if not self.voxels:
            return
        voxel_size = float(self.get_parameter("voxel_size").value)
        keys = np.array(list(self.voxels.keys()), dtype=np.float32)
        temperatures = np.fromiter(
            self.voxels.values(), dtype=np.float32, count=len(self.voxels)
        )
        centres = (keys + 0.5) * voxel_size
        colors = temperature_colors(
            temperatures,
            float(self.get_parameter("min_temp_c").value),
            float(self.get_parameter("max_temp_c").value),
        )
        self.publisher.publish(
            cloud_message(
                centres,
                colors,
                self.get_parameter("map_frame").value,
                self.get_clock().now().to_msg(),
            )
        )


def main() -> None:
    rclpy.init()
    node = ThermalCloud()
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
