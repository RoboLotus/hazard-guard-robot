from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from .battery import BatteryCalibration, VoltageSmoother


class BatteryTelemetryNode(Node):
    """Normalize Yahboom's ``/voltage`` topic as a standard BatteryState."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_battery_telemetry")
        self.declare_parameter("input_topic", "/voltage")
        self.declare_parameter("output_topic", "/hazard_guard/battery")
        self.declare_parameter("window_size", 20)
        self.declare_parameter("empty_voltage_v", 10.5)
        self.declare_parameter("full_voltage_v", 12.6)

        self._calibration = BatteryCalibration(
            empty_voltage_v=float(self.get_parameter("empty_voltage_v").value),
            full_voltage_v=float(self.get_parameter("full_voltage_v").value),
        )
        self._smoother = VoltageSmoother(
            int(self.get_parameter("window_size").value)
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(BatteryState, output_topic, 10)
        self.create_subscription(
            Float32,
            str(self.get_parameter("input_topic").value),
            self._on_voltage,
            10,
        )
        self.get_logger().info(
            "Yahboom battery telemetry: /voltage -> " + output_topic
        )

    def _on_voltage(self, message: Float32) -> None:
        try:
            voltage_v = self._smoother.observe(float(message.data))
            percentage = self._calibration.percentage(voltage_v)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid battery voltage: {exc}")
            return

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = float(voltage_v)
        battery.percentage = float(percentage)
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.present = True
        battery.location = "rosmaster-m1-main"
        self._publisher.publish(battery)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryTelemetryNode()
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
