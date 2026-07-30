from __future__ import annotations

import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2SmokeTest(Node):
    """Send one nearby Nav2 goal and return a process-friendly result."""

    def __init__(self) -> None:
        super().__init__("hazard_guard_nav2_smoke_test")
        self.declare_parameter("target_x", 0.95)
        self.declare_parameter("target_y", 0.63)
        self.declare_parameter("target_yaw_z", 0.0)
        self.declare_parameter("target_yaw_w", 1.0)
        self.declare_parameter("server_timeout_sec", 10.0)
        self.declare_parameter("result_timeout_sec", 45.0)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def run(self) -> int:
        server_timeout = float(self.get_parameter("server_timeout_sec").value)
        if not self.client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("Nav2 /navigate_to_pose action server is unavailable.")
            return 2

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("target_x").value)
        goal.pose.pose.position.y = float(self.get_parameter("target_y").value)
        goal.pose.pose.orientation.z = float(
            self.get_parameter("target_yaw_z").value
        )
        goal.pose.pose.orientation.w = float(
            self.get_parameter("target_yaw_w").value
        )

        self.get_logger().info(
            "Sending Nav2 smoke-test goal "
            f"(x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f})."
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=server_timeout)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Timed out while sending the Nav2 goal.")
            return 3

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the smoke-test goal.")
            return 4

        result_future = goal_handle.get_result_async()
        result_timeout = float(self.get_parameter("result_timeout_sec").value)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Nav2 goal did not finish before the timeout.")
            goal_handle.cancel_goal_async()
            return 5

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Nav2 goal finished with status={status}.")
            return 6

        self.get_logger().info("Nav2 smoke-test goal succeeded.")
        return 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2SmokeTest()
    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
