from __future__ import annotations

import math

import rclpy
from hazard_guard_interfaces.msg import HazardDetection
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .perception import visible_heat_sources


class ThermalDetectorMock(Node):
    """Publish deterministic map-space heat sources visible to the robot."""

    HEAT_SOURCES = [
        {
            "detection_id": "sim-hot-motor",
            "x": -0.2,
            "y": -2.5,
            "z": 0.55,
            "temperature_c": 84.6,
            "radius_m": 0.42,
            "source": "gazebo:hot_motor",
        },
        {
            "detection_id": "sim-pump-block",
            "x": 1.8,
            "y": 1.2,
            "z": 0.65,
            "temperature_c": 68.4,
            "radius_m": 0.5,
            "source": "gazebo:pump_block",
        },
        {
            "detection_id": "sim-tank-block",
            "x": -1.8,
            "y": 1.8,
            "z": 0.8,
            "temperature_c": 48.2,
            "radius_m": 0.55,
            "source": "gazebo:tank_block",
        },
    ]

    def __init__(self) -> None:
        super().__init__("hazard_guard_thermal_detector_mock")
        self.declare_parameter("camera_model", "ThermoEye TMC160B")
        self.declare_parameter("horizontal_fov_deg", 57.0)
        self.declare_parameter("range_min_m", 0.0)
        self.declare_parameter("range_max_m", 5.0)
        self.declare_parameter("sensor_frame", "thermal_camera_link")
        self.declare_parameter("publish_rate_hz", 2.0)
        self._publisher = self.create_publisher(
            HazardDetection,
            "/hazard_guard/thermal_detections",
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        publish_rate = max(
            0.2,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_visible)
        self.get_logger().info(
            "Synthetic thermal detector configured for "
            f"{self.get_parameter('camera_model').value}; "
            "5 m is a visualization boundary, not a hardware range claim."
        )

    def _publish_visible(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                str(self.get_parameter("sensor_frame").value),
                Time(),
            )
        except Exception:
            return

        position = transform.transform.translation
        orientation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        visible = visible_heat_sources(
            float(position.x),
            float(position.y),
            yaw,
            self.HEAT_SOURCES,
            horizontal_fov_deg=float(
                self.get_parameter("horizontal_fov_deg").value
            ),
            range_min_m=float(self.get_parameter("range_min_m").value),
            range_max_m=float(self.get_parameter("range_max_m").value),
        )
        stamp = self.get_clock().now().to_msg()
        for source in visible:
            message = HazardDetection()
            message.stamp = stamp
            message.frame_id = "map"
            message.detection_id = source["detection_id"]
            message.x = float(source["x"])
            message.y = float(source["y"])
            message.z = float(source["z"])
            message.temperature_c = float(source["temperature_c"])
            message.confidence = float(source["confidence"])
            message.radius_m = float(source["radius_m"])
            message.source = source["source"]
            message.simulated = True
            self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThermalDetectorMock()
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
