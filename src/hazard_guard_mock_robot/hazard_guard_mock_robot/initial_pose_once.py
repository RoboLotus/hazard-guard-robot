from __future__ import annotations

from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class InitialPosePublisher(Node):
    """Publish a finite burst of initial-pose messages for Nav2 AMCL."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_initial_pose")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("interval_sec", 0.5)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )
        self._remaining = max(1, int(self.get_parameter("repeat_count").value))
        interval_sec = max(0.1, float(self.get_parameter("interval_sec").value))
        self._timer = self.create_timer(interval_sec, self._publish)

    def _publish(self) -> None:
        message = PoseWithCovarianceStamped()
        # Keep the timestamp at zero so AMCL asks tf2 for the latest odom
        # transform. A clock-stamped pose can be rejected during startup when
        # Gazebo /clock and the odom TF cache are not aligned yet.
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(self.get_parameter("x").value)
        message.pose.pose.position.y = float(self.get_parameter("y").value)

        yaw = float(self.get_parameter("yaw").value)
        message.pose.pose.orientation.z = sin(yaw / 2.0)
        message.pose.pose.orientation.w = cos(yaw / 2.0)

        # Conservative planar uncertainty: x, y, and yaw.
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685

        self._publisher.publish(message)
        self._remaining -= 1
        self.get_logger().info(
            "Published AMCL initial pose "
            f"(x={message.pose.pose.position.x:.2f}, "
            f"y={message.pose.pose.position.y:.2f}, yaw={yaw:.2f}); "
            f"{self._remaining} retries remain."
        )
        if self._remaining == 0:
            self._timer.cancel()
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
