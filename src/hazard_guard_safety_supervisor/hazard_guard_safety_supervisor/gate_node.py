"""Final fail-safe gate between Nav2/teleop velocity and the motor driver."""

import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from hazard_guard_interfaces.msg import PersonSafetyState
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .gate import command_path_is_allowed
from .policy import SafetyState


class CmdVelSafetyGateNode(Node):
    """Publish either the original velocity or an authoritative zero command."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("cmd_vel_safety_gate", **node_kwargs)
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter(
            "safety_state_topic",
            "/hazard_guard/person/safety_state",
        )
        self.declare_parameter("safety_state_timeout_sec", 0.75)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("zero_publish_rate_hz", 10.0)
        self.declare_parameter("require_safety_state", True)
        self.declare_parameter("shutdown_zero_count", 5)
        self.declare_parameter("shutdown_zero_interval_sec", 0.02)
        self.declare_parameter("shutdown_settle_sec", 0.10)

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
            self._blocked = not command_path_is_allowed(
                state,
                state_fresh=True,
                command_fresh=self._command_is_fresh(now_sec),
                require_safety_state=self._requires_safety_state(),
            )
        if self._blocked and not was_blocked:
            self._publisher.publish(Twist())

    def _on_velocity(self, message: Twist) -> None:
        with self._lock:
            now_sec = self._now_sec()
            self._last_command_sec = now_sec
            fresh = self._state_is_fresh(now_sec)
            allowed = command_path_is_allowed(
                self._state,
                state_fresh=fresh,
                command_fresh=True,
                require_safety_state=self._requires_safety_state(),
            )
            self._blocked = not allowed
        self._publisher.publish(message if allowed else Twist())

    def _command_is_fresh(self, now_sec: float) -> bool:
        return (
            self._last_command_sec is not None
            and now_sec - self._last_command_sec
            <= self._float_parameter("cmd_vel_timeout_sec")
        )

    def _requires_safety_state(self) -> bool:
        return bool(self.get_parameter("require_safety_state").value)

    def _publish_zero_if_blocked(self) -> None:
        with self._lock:
            now_sec = self._now_sec()
            state_fresh = self._state_is_fresh(now_sec)
            command_fresh = self._command_is_fresh(now_sec)
            blocked = not command_path_is_allowed(
                self._state,
                state_fresh=state_fresh,
                command_fresh=command_fresh,
                require_safety_state=self._requires_safety_state(),
            )
            self._blocked = blocked
        if blocked:
            self._publisher.publish(Twist())

    def publish_stop_burst(
        self,
        *,
        zero_count: int | None = None,
        interval_sec: float | None = None,
        settle_sec: float | None = None,
    ) -> None:
        """Send repeated motor-facing zeros before DDS and driver teardown."""

        count = max(
            3,
            int(
                self.get_parameter("shutdown_zero_count").value
                if zero_count is None
                else zero_count
            ),
        )
        interval = max(
            0.0,
            float(
                self.get_parameter("shutdown_zero_interval_sec").value
                if interval_sec is None
                else interval_sec
            ),
        )
        settle = max(
            0.0,
            float(
                self.get_parameter("shutdown_settle_sec").value
                if settle_sec is None
                else settle_sec
            ),
        )
        for index in range(count):
            self._publisher.publish(Twist())
            if interval and index + 1 < count:
                time.sleep(interval)
        if settle:
            time.sleep(settle)


def main(args=None) -> None:
    # Own SIGINT/SIGTERM so the DDS context remains alive until the final zero
    # burst has had time to reach the motor driver.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = CmdVelSafetyGateNode()
    shutdown_requested = threading.Event()
    previous_handlers = {}

    def request_shutdown(_signum, _frame) -> None:
        if shutdown_requested.is_set():
            return
        shutdown_requested.set()
        # The launch service signals all child processes during teardown. Send
        # motor-facing zeros immediately, while this DDS context is certainly
        # still alive, instead of waiting for the spin loop to unwind.
        if rclpy.ok():
            node.publish_stop_burst()

    for shutdown_signal in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[shutdown_signal] = signal.getsignal(
            shutdown_signal
        )
        signal.signal(shutdown_signal, request_shutdown)
    try:
        while rclpy.ok() and not shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Repeat the burst after spin has stopped for redundancy, and only
        # then tear down the publisher/context.
        if rclpy.ok():
            node.publish_stop_burst()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for shutdown_signal, previous in previous_handlers.items():
            signal.signal(shutdown_signal, previous)
