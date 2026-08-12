from __future__ import annotations

import math
import json
from pathlib import Path

import rclpy
from hazard_guard_interfaces.msg import HazardDetection
from hazard_guard_sensor_config import TMC160B
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .perception import visible_heat_sources


class ThermalDetectorMock(Node):
    """Publish deterministic map-space heat sources visible to the robot."""

    # Map-space positions on the classroom demo world. The equipment sits as an
    # island in the middle of the hall and the robot patrols the ring around
    # it, so each source is put on the island face that turns toward the ring
    # segment it is read from - see the waypoint list in tools/run_patrol.sh.
    #
    # These match worlds/demo_facility_scaled.sdf, where the process line is
    # placed at scale 0.07474982, and the radii are scaled with it. The same
    # values are mirrored in config/heat_sources/demo_facility_scaled.json;
    # that file is not loaded yet, so both have to be edited together until a
    # loader exists. Move them with the layout if the world is regenerated.
    HEAT_SOURCES = [
        {
            "detection_id": "sim-hot-motor",
            "x": -1.1418,
            "y": -0.4065,
            "z": 0.2548,
            "temperature_c": 84.6,
            "radius_m": 0.057,
            "source": "gazebo:primary_shredder_motor",
        },
        {
            "detection_id": "sim-pump-block",
            "x": 0.8514,
            "y": -0.3046,
            "z": 0.2548,
            "temperature_c": 68.4,
            "radius_m": 0.057,
            "source": "gazebo:secondary_processor_pump",
        },
        {
            "detection_id": "sim-tank-block",
            "x": 1.344,
            "y": -0.3386,
            "z": 0.1982,
            "temperature_c": 48.2,
            "radius_m": 0.051,
            "source": "gazebo:baler_hydraulic_tank",
        },
        {
            "detection_id": "sim-waste-pile",
            "x": -1.5098,
            "y": -0.718,
            "z": 0.2548,
            "temperature_c": 71.3,
            "radius_m": 0.074,
            "source": "gazebo:bunker_waste_pile",
        },
    ]

    def __init__(self) -> None:
        super().__init__("hazard_guard_thermal_detector_mock")
        self.declare_parameter("camera_model", TMC160B.model)
        self.declare_parameter(
            "horizontal_fov_deg",
            TMC160B.horizontal_fov_deg,
        )
        self.declare_parameter("range_min_m", 0.0)
        self.declare_parameter(
            "range_max_m",
            TMC160B.visualization_range_m,
        )
        self.declare_parameter("sensor_frame", TMC160B.sensor_frame)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("heat_source_profile", "")
        self._heat_sources = self._load_heat_sources(
            str(self.get_parameter("heat_source_profile").value)
        )
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
        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_visible,
        )
        self.get_logger().info(
            "Synthetic thermal detector configured for "
            f"{self.get_parameter('camera_model').value}; "
            f"{TMC160B.visualization_range_m:g} m is a "
            "visualization boundary, "
            "not a hardware range claim."
        )

    def _load_heat_sources(self, profile_value: str) -> list[dict]:
        if not profile_value:
            return list(self.HEAT_SOURCES)
        profile_path = Path(profile_value).expanduser()
        try:
            document = json.loads(profile_path.read_text(encoding="utf-8"))
            sources = document.get("sources", [])
            required = {
                "detection_id",
                "x",
                "y",
                "z",
                "temperature_c",
                "radius_m",
                "source",
            }
            if not isinstance(sources, list) or not sources:
                raise ValueError("sources must be a non-empty list")
            if any(not isinstance(item, dict) or not required <= item.keys() for item in sources):
                raise ValueError("a heat source is missing required fields")
            self.get_logger().info(
                f"Loaded {len(sources)} synthetic heat sources from {profile_path.name}"
            )
            return sources
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"Could not load heat source profile '{profile_value}': {exc}; "
                "using built-in synthetic sources"
            )
            return list(self.HEAT_SOURCES)

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
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        visible = visible_heat_sources(
            float(position.x),
            float(position.y),
            yaw,
            self._heat_sources,
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
