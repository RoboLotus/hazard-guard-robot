from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from .model import FirstOrderGasSensor, FirstOrderThermalSource, GasScenario


class GasSensorSimulator(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_gas_sensor_simulator")
        default_config = (
            Path(get_package_share_directory("hazard_guard_gas_monitor"))
            / "config"
            / "demo_gas_scenario.json"
        )
        self.declare_parameter("scenario_path", str(default_config))
        self.declare_parameter("publish_rate_hz", 2.0)
        self.scenario = GasScenario.load(
            str(self.get_parameter("scenario_path").value)
        )
        self.sensor = FirstOrderGasSensor(
            self.scenario.ambient, self.scenario.sensor
        )
        self.thermal_source = FirstOrderThermalSource(
            self.scenario.timeline[0].surface_temperature_c,
            self.scenario.sensor.thermal_time_constant_sec,
        )
        self._position = (0.0975, -1.4121)
        self._fan_on = False
        self._started = time.monotonic()
        self._last_update = self._started
        self._publisher = self.create_publisher(
            String, "/hazard_guard/gas/reading", 10
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._incident_publisher = self.create_publisher(
            String, "/hazard_guard/incident/battery/status", status_qos
        )
        self._temperature_publisher = self.create_publisher(
            Float64, "/hazard_guard/incident/battery/temperature", 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            String, "/hazard_guard/gas/control", self._on_control, 10
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Gas sensor simulation ready: {self.scenario.source.name}")

    def _on_odom(self, message: Odometry) -> None:
        self._position = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))

    def _on_control(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if "fan_on" in payload:
            self._fan_on = bool(payload["fan_on"])

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._started
        dt = now - self._last_update
        self._last_update = now
        phase, target, distance = self.scenario.concentration_at(*self._position, elapsed)
        phase_config = self.scenario.phase_at(elapsed)
        measured = self.sensor.update(target, dt, self._fan_on)
        surface_temperature_c = self.thermal_source.update(
            phase_config.surface_temperature_c,
            dt,
        )
        incident = {
            "schema_version": 1,
            "source_id": self.scenario.source.source_id,
            "source_name": self.scenario.source.name,
            "equipment_id": self.scenario.source.equipment_id,
            "visual_model_id": self.scenario.source.visual_model_id,
            "frame_id": self.scenario.frame_id,
            "x": self.scenario.source.x,
            "y": self.scenario.source.y,
            "phase": phase,
            "target_surface_temperature_c": round(
                phase_config.surface_temperature_c, 2
            ),
            "surface_temperature_c": round(surface_temperature_c, 2),
            "elapsed_sec": round(elapsed, 3),
            "simulated": True,
        }
        incident_message = String()
        incident_message.data = json.dumps(incident, ensure_ascii=False)
        self._incident_publisher.publish(incident_message)
        temperature_message = Float64()
        temperature_message.data = surface_temperature_c + 273.15
        self._temperature_publisher.publish(temperature_message)
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "frame_id": self.scenario.frame_id,
                "x": round(self._position[0], 4),
                "y": round(self._position[1], 4),
                "voc_index": round(measured.voc_index, 2),
                "co_ppm": round(measured.co_ppm, 3),
                "co2_ppm": round(measured.co2_ppm, 1),
                "phase": phase,
                "surface_temperature_c": round(surface_temperature_c, 2),
                "target_surface_temperature_c": round(
                    phase_config.surface_temperature_c, 2
                ),
                "source_id": self.scenario.source.source_id,
                "source_name": self.scenario.source.name,
                "equipment_id": self.scenario.source.equipment_id,
                "visual_model_id": self.scenario.source.visual_model_id,
                "source_distance_m": round(distance, 3),
                "fan_on": self._fan_on,
                "warmed_up": elapsed >= self.scenario.sensor.warmup_sec,
                "elapsed_sec": round(elapsed, 3),
                "simulated": True,
            },
            ensure_ascii=False,
        )
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GasSensorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
