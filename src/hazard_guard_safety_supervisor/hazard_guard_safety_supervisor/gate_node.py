"""Final fail-safe gate between Nav2/teleop velocity and the motor driver."""

import threading

import rclpy
from geometry_msgs.msg import Twist
from hazard_guard_interfaces.msg import PersonSafetyState
from rclpy.node import Node

from .gate import motion_is_allowed
from .policy import SafetyState


class CmdVelSafetyGateNode(Node):
    """Publish either the original velocity or an authoritative zero command."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_safety_gate")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter(
            "safety_state_topic",
            "/hazard_guard/person/safety_state",
        )
        self.declare_parameter("safety_state_timeout_sec", 0.75)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("zero_publish_rate_hz", 10.0)

        self._lock = threading.RLock()
        self._state = SafetyState.SENSOR_FAULT
        self._last_state_sec: float | None = None
        self._last_command_sec: float | None = None
        self._blocked = True

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        state_topic = str(self.get_parameter("safety_state_topic").value)
        timeout = self._float_parameter("safety_state_timeout_sec")
        publish_rate = self._float_parameter("zero_publish_rate_hz")
        command_timeout = self._float_parameter("cmd_vel_timeout_sec")
        if timeout <= 0.0 or command_timeout <= 0.0 or publish_rate <= 0.0:
            raise ValueError("gate timeouts and publish rate must be positive")

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._velocity_subscription = self.create_subscription(
            Twist,
            input_topic,
            self._on_velocity,
            10,
        )
        self._state_subscription = self.create_subscription(
            PersonSafetyState,
            state_topic,
            self._on_safety_state,
            10,
        )
        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_zero_if_blocked,
        )
        self.get_logger().info(
            f"fail-safe cmd_vel gate: {input_topic} -> {output_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _state_is_fresh(self, now_sec: float) -> bool:
        return (
            self._last_state_sec is not None
            and now_sec - self._last_state_sec
            <= self._float_parameter("safety_state_timeout_sec")
        )

    def _on_safety_state(self, message: PersonSafetyState) -> None:
        try:
            state = SafetyState(int(message.state))
        except ValueError:
            state = SafetyState.SENSOR_FAULT
        with self._lock:
            was_blocked = self._blocked
            now_sec = self._now_sec()
            self._state = state
            self._last_state_sec = now_sec
            self._blocked = not motion_is_allowed(
                state,
                state_fresh=True,
                command_fresh=self._command_is_fresh(now_sec),
            )
        if self._blocked and not was_blocked:
            self._publisher.publish(Twist())

    def _on_velocity(self, message: Twist) -> None:
        with self._lock:
            now_sec = self._now_sec()
            self._last_command_sec = now_sec
            fresh = self._state_is_fresh(now_sec)
            allowed = motion_is_allowed(
                self._state,
                state_fresh=fresh,
                command_fresh=True,
            )
            self._blocked = not allowed
        self._publisher.publish(message if allowed else Twist())

    def _command_is_fresh(self, now_sec: float) -> bool:
        return (
            self._last_command_sec is not None
            and now_sec - self._last_command_sec
            <= self._float_parameter("cmd_vel_timeout_sec")
        )

    def _publish_zero_if_blocked(self) -> None:
        with self._lock:
            now_sec = self._now_sec()
            state_fresh = self._state_is_fresh(now_sec)
            command_fresh = self._command_is_fresh(now_sec)
            blocked = not motion_is_allowed(
                self._state,
                state_fresh=state_fresh,
                command_fresh=command_fresh,
            )
            self._blocked = blocked
        if blocked:
            self._publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
