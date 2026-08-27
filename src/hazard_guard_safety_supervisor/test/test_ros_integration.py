import threading
import time

from geometry_msgs.msg import Twist
from hazard_guard_interfaces.msg import (
    PersonObservation,
    PersonObservationArray,
    PersonSafetyState,
)
from nav2_msgs.msg import SpeedLimit
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from hazard_guard_safety_supervisor.gate_node import CmdVelSafetyGateNode
from hazard_guard_safety_supervisor.node import PersonSafetySupervisorNode


def _wait_until(predicate, timeout_sec: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for ROS safety integration state")


def _observation(distance_m: float) -> PersonObservationArray:
    message = PersonObservationArray()
    message.rgb_valid = True
    message.depth_valid = True
    message.health_reason = "test RGB-D pair is healthy"
    person = PersonObservation()
    person.confidence = 0.9
    person.distance_m = distance_m
    person.distance_valid = True
    message.observations.append(person)
    return message


def test_observation_drives_nav2_limit_and_final_velocity_gate() -> None:
    rclpy.init()
    supervisor = PersonSafetySupervisorNode()
    gate = CmdVelSafetyGateNode()
    probe = Node("person_safety_integration_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (supervisor, gate, probe):
        executor.add_node(node)

    states = []
    limits = []
    safe_commands = []
    probe.create_subscription(
        PersonSafetyState,
        "/hazard_guard/person/safety_state",
        states.append,
        10,
    )
    probe.create_subscription(SpeedLimit, "/speed_limit", limits.append, 10)
    probe.create_subscription(Twist, "/cmd_vel_safe", safe_commands.append, 10)
    observations = probe.create_publisher(
        PersonObservationArray,
        "/hazard_guard/person/observations",
        10,
    )
    velocity = probe.create_publisher(Twist, "/cmd_vel", 10)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        for _ in range(5):
            observations.publish(_observation(1.4))
            time.sleep(0.05)
        _wait_until(
            lambda: states and states[-1].state == PersonSafetyState.SLOW
        )
        _wait_until(
            lambda: limits
            and limits[-1].percentage
            and limits[-1].speed_limit == 45.0
        )

        safe_commands.clear()
        command = Twist()
        command.linear.x = 0.2
        for _ in range(3):
            velocity.publish(command)
            time.sleep(0.05)
        _wait_until(
            lambda: any(message.linear.x == 0.2 for message in safe_commands)
        )

        # If Nav2/velocity_smoother dies after a non-zero command, the final
        # gate must actively overwrite the last motor command with zero.
        safe_commands.clear()
        _wait_until(
            lambda: safe_commands and safe_commands[-1].linear.x == 0.0,
            timeout_sec=1.5,
        )

        safe_commands.clear()
        gate.publish_stop_burst(
            zero_count=3,
            interval_sec=0.02,
            settle_sec=0.05,
        )
        _wait_until(lambda: len(safe_commands) >= 3)
        assert all(
            message.linear.x == 0.0 and message.angular.z == 0.0
            for message in safe_commands
        )

        observations.publish(_observation(0.5))
        _wait_until(
            lambda: states and states[-1].state == PersonSafetyState.STOP
        )
        safe_commands.clear()
        velocity.publish(command)
        _wait_until(lambda: safe_commands)
        assert safe_commands[-1].linear.x == 0.0
        assert safe_commands[-1].angular.z == 0.0
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        for node in (probe, gate, supervisor):
            node.destroy_node()
        rclpy.shutdown()


def test_depth_health_failure_becomes_sensor_fault() -> None:
    rclpy.init()
    supervisor = PersonSafetySupervisorNode()
    probe = Node("person_safety_health_probe")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(supervisor)
    executor.add_node(probe)
    states = []
    probe.create_subscription(
        PersonSafetyState,
        "/hazard_guard/person/safety_state",
        states.append,
        10,
    )
    publisher = probe.create_publisher(
        PersonObservationArray,
        "/hazard_guard/person/observations",
        10,
    )
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        message = PersonObservationArray()
        message.rgb_valid = True
        message.depth_valid = False
        message.health_reason = "depth registration is not verified"
        for _ in range(3):
            publisher.publish(message)
            time.sleep(0.05)
        _wait_until(
            lambda: states
            and states[-1].state == PersonSafetyState.SENSOR_FAULT
        )
        assert states[-1].reason == "depth registration is not verified"
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        probe.destroy_node()
        supervisor.destroy_node()
        rclpy.shutdown()


def test_watchdog_mode_stops_motor_topic_when_nav2_goes_silent() -> None:
    rclpy.init()
    gate = CmdVelSafetyGateNode(
        parameter_overrides=[
            Parameter("require_safety_state", value=False),
        ]
    )
    probe = Node("cmd_vel_watchdog_integration_probe")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gate)
    executor.add_node(probe)
    safe_commands = []
    probe.create_subscription(
        Twist,
        "/cmd_vel_safe",
        safe_commands.append,
        10,
    )
    velocity = probe.create_publisher(Twist, "/cmd_vel", 10)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        command = Twist()
        command.linear.x = 0.2
        for _ in range(3):
            velocity.publish(command)
            time.sleep(0.05)
        _wait_until(
            lambda: any(message.linear.x == 0.2 for message in safe_commands)
        )

        safe_commands.clear()
        _wait_until(
            lambda: safe_commands and safe_commands[-1].linear.x == 0.0,
            timeout_sec=1.5,
        )
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        probe.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
