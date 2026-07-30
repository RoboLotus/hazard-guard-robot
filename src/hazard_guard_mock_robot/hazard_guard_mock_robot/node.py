from __future__ import annotations

from math import hypot
from time import monotonic

import rclpy
from hazard_guard_interfaces.msg import RobotTelemetry
from hazard_guard_interfaces.srv import RobotCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .state import RobotState


class MockRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_mock_robot")
        self.declare_parameter("robot_id", "rosmaster-m1-mock")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("initial_battery_percent", 78.0)
        self.declare_parameter("use_simulation_inputs", False)
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")

        self._state = RobotState(
            robot_id=str(self.get_parameter("robot_id").value),
            battery_percent=float(self.get_parameter("initial_battery_percent").value),
        )
        self._started_at = monotonic()
        self._last_scan_at: float | None = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(RobotTelemetry, "/hazard_guard/telemetry", qos)
        self._service = self.create_service(
            RobotCommand,
            "/hazard_guard/command",
            self._handle_command,
        )
        rate_hz = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_telemetry)
        if bool(self.get_parameter("use_simulation_inputs").value):
            self._state.lidar_status = "waiting"
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odometry_topic").value),
                self._handle_odometry,
                qos,
            )
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._handle_scan,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "Simulation telemetry inputs enabled for odometry and LiDAR."
            )
        self.get_logger().info("HazardGuard mock robot started; no hardware commands will be sent.")

    def _handle_odometry(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        self._state.measured_speed_mps = round(hypot(velocity.x, velocity.y), 3)

    def _handle_scan(self, _: LaserScan) -> None:
        now = monotonic()
        if self._last_scan_at is not None:
            period = now - self._last_scan_at
            if period > 0:
                measured_hz = min(100.0, 1.0 / period)
                self._state.lidar_hz = round(
                    0.8 * self._state.lidar_hz + 0.2 * measured_hz, 2
                )
        self._last_scan_at = now
        self._state.lidar_status = "normal"

    def _publish_telemetry(self) -> None:
        if self._last_scan_at is not None and monotonic() - self._last_scan_at > 2.0:
            self._state.lidar_status = "offline"
        snapshot = self._state.snapshot(monotonic() - self._started_at)
        message = RobotTelemetry()
        message.stamp = self.get_clock().now().to_msg()
        message.robot_id = str(snapshot["robot_id"])
        message.mode = str(snapshot["mode"])
        message.battery_percent = float(snapshot["battery_percent"])
        message.speed_mps = float(snapshot["speed_mps"])
        message.network_quality = str(snapshot["network_quality"])
        message.network_rssi_dbm = int(snapshot["network_rssi_dbm"])
        message.lidar_status = str(snapshot["lidar_status"])
        message.lidar_hz = float(snapshot["lidar_hz"])
        message.max_temperature_c = float(snapshot["max_temperature_c"])
        message.alert_level = str(snapshot["alert_level"])
        message.controller_enabled = bool(snapshot["controller_enabled"])
        message.mock = True
        self._publisher.publish(message)

    def _handle_command(self, request: RobotCommand.Request, response: RobotCommand.Response):
        accepted, detail = self._state.apply_command(request.command, request.enabled)
        response.accepted = accepted
        response.message = detail
        response.mode = self._state.mode
        response.controller_enabled = self._state.controller_enabled
        response.mock = True
        log = self.get_logger().info if accepted else self.get_logger().warning
        log(f"mock command '{request.command}': {detail}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockRobotNode()
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
