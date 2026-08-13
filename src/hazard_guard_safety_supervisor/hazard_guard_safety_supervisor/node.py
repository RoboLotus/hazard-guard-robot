"""ROS adapter for the pure person-safety policy."""

import math
from typing import Iterable

import rclpy
from rclpy.node import Node

from hazard_guard_interfaces.msg import PersonObservationArray, PersonSafetyState
from nav2_msgs.msg import SpeedLimit

from .policy import PersonObservation, PersonSafetyPolicy, PolicyConfig
from .speed_limit import speed_limit_for_state


class PersonSafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("person_safety_supervisor")

        self.declare_parameter("observation_topic", "/hazard_guard/person/observations")
        self.declare_parameter("safety_state_topic", "/hazard_guard/person/safety_state")
        self.declare_parameter("speed_limit_topic", "/speed_limit")
        self.declare_parameter("autonomous", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("caution_distance_m", 2.5)
        self.declare_parameter("slow_distance_m", 1.8)
        self.declare_parameter("stop_distance_m", 0.9)
        self.declare_parameter("minimum_confidence", 0.4)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("clear_hold_sec", 2.0)
        self.declare_parameter("hysteresis_m", 0.2)
        self.declare_parameter("slow_speed_percentage", 45.0)
        self.declare_parameter("restrictive_speed_percentage", 1.0)

        self._policy = PersonSafetyPolicy(
            PolicyConfig(
                caution_distance_m=self._float_parameter("caution_distance_m"),
                slow_distance_m=self._float_parameter("slow_distance_m"),
                stop_distance_m=self._float_parameter("stop_distance_m"),
                minimum_confidence=self._float_parameter("minimum_confidence"),
                detection_timeout_sec=self._float_parameter("detection_timeout_sec"),
                clear_hold_sec=self._float_parameter("clear_hold_sec"),
                hysteresis_m=self._float_parameter("hysteresis_m"),
            )
        )

        observation_topic = str(self.get_parameter("observation_topic").value)
        safety_state_topic = str(self.get_parameter("safety_state_topic").value)
        speed_limit_topic = str(self.get_parameter("speed_limit_topic").value)
        publish_rate_hz = self._float_parameter("publish_rate_hz")
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._latest_observations: tuple[PersonObservation, ...] = ()
        self._last_detector_update_sec: float | None = None
        self._sensor_healthy = False
        self._sensor_reason = "no RGB-D health report received"
        self._publisher = self.create_publisher(PersonSafetyState, safety_state_topic, 10)
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit,
            speed_limit_topic,
            10,
        )
        self._subscription = self.create_subscription(
            PersonObservationArray,
            observation_topic,
            self._on_observations,
            10,
        )
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_state)

        self.get_logger().info(
            f"person safety supervisor: {observation_topic} -> {safety_state_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_observations(self, message: PersonObservationArray) -> None:
        self._latest_observations = tuple(self._convert_observations(message))
        self._last_detector_update_sec = self._now_sec()
        self._sensor_healthy = bool(message.rgb_valid and message.depth_valid)
        self._sensor_reason = str(message.health_reason)
        self._publish_state()

    @staticmethod
    def _convert_observations(
        message: PersonObservationArray,
    ) -> Iterable[PersonObservation]:
        # ``observations`` is the canonical contract. The fallbacks make the
        # adapter tolerant of an early interface rename without weakening the
        # pure policy or its tests.
        values = getattr(
            message,
            "observations",
            getattr(message, "detections", getattr(message, "persons", ())),
        )
        for value in values:
            distance_m = float(
                getattr(value, "distance_m", getattr(value, "distance", math.nan))
            )
            explicit_validity = getattr(value, "distance_valid", None)
            distance_valid = (
                bool(explicit_validity)
                if explicit_validity is not None
                else math.isfinite(distance_m) and distance_m >= 0.0
            )
            yield PersonObservation(
                confidence=float(getattr(value, "confidence", 0.0)),
                distance_m=distance_m,
                distance_valid=distance_valid,
            )

    def _publish_state(self) -> None:
        result = self._policy.evaluate(
            now_sec=self._now_sec(),
            observations=self._latest_observations,
            last_detector_update_sec=self._last_detector_update_sec,
            autonomous=bool(self.get_parameter("autonomous").value),
            sensor_healthy=self._sensor_healthy,
            sensor_reason=self._sensor_reason,
        )

        message = PersonSafetyState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.state = int(result.state)
        message.state_name = result.state.name
        message.person_count = result.person_count
        message.nearest_distance_m = float(result.nearest_distance_m)
        message.distance_valid = result.distance_valid
        message.detector_stale = result.detector_stale
        message.reason = result.reason
        self._publisher.publish(message)

        decision = speed_limit_for_state(
            result.state,
            slow_percentage=self._float_parameter("slow_speed_percentage"),
            restrictive_percentage=self._float_parameter(
                "restrictive_speed_percentage"
            ),
        )
        speed_limit = SpeedLimit()
        speed_limit.header.stamp = message.header.stamp
        speed_limit.header.frame_id = "base_link"
        speed_limit.percentage = decision.percentage
        speed_limit.speed_limit = decision.speed_limit
        self._speed_limit_publisher.publish(speed_limit)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonSafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
