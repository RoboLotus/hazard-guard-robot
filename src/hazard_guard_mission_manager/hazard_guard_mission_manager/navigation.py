from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .errors import MissionCanceled, MissionFailure, MissionSafetyPaused
from .geometry import path_length


class Nav2Adapter:
    """Small boundary around Nav2 actions and TF used by patrol execution."""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        *,
        navigate_action_name: str,
        compute_path_action_name: str,
        spin_action_name: str,
        base_frame: str,
        server_wait_timeout_sec: float,
        check_canceled: Callable[[Any], None],
        safety_is_paused: Callable[[], bool],
    ) -> None:
        self._node = node
        self._base_frame = base_frame
        self._server_wait_timeout_sec = server_wait_timeout_sec
        self._check_canceled = check_canceled
        self._safety_is_paused = safety_is_paused
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )
        self._navigate_client = ActionClient(
            node,
            NavigateToPose,
            navigate_action_name,
            callback_group=callback_group,
        )
        self._path_client = ActionClient(
            node,
            ComputePathToPose,
            compute_path_action_name,
            callback_group=callback_group,
        )
        self._spin_client = ActionClient(
            node,
            Spin,
            spin_action_name,
            callback_group=callback_group,
        )
        self._active_goal_lock = threading.RLock()
        self._active_nav_goal: Any | None = None
        self._safety_cancel_requested = False

    def assert_ready(self) -> None:
        if not self._path_client.wait_for_server(
            timeout_sec=self._server_wait_timeout_sec
        ):
            raise MissionFailure("Nav2 경로 계산 서버에 연결할 수 없습니다.")
        if not self._navigate_client.wait_for_server(
            timeout_sec=self._server_wait_timeout_sec
        ):
            raise MissionFailure("Nav2 이동 서버에 연결할 수 없습니다.")
        if not self._spin_client.wait_for_server(
            timeout_sec=self._server_wait_timeout_sec
        ):
            raise MissionFailure("Nav2 제자리 선회 서버에 연결할 수 없습니다.")

    def current_pose(self, frame_id: str) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id or "map",
                self._base_frame,
                Time(),
            )
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return float(translation.x), float(translation.y), yaw

    def compute_path_distance(
        self,
        start: tuple[float, float, float],
        target: tuple[float, float, float],
        frame_id: str,
        mission_goal: Any,
    ) -> float:
        goal = ComputePathToPose.Goal()
        goal.start = self._pose_stamped(start, frame_id)
        goal.goal = self._pose_stamped(target, frame_id)
        goal.use_start = True
        goal.planner_id = "GridBased"
        nav_goal = self._send_goal(self._path_client, goal, mission_goal, 15.0)
        wrapped = self._wait_result(nav_goal, mission_goal, 30.0)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionFailure("Nav2가 웨이포인트 경로를 생성하지 못했습니다.")
        poses = list(wrapped.result.path.poses)
        if len(poses) < 2:
            if math.hypot(target[0] - start[0], target[1] - start[1]) < 0.08:
                return 0.0
            raise MissionFailure(
                "목적지까지 유효한 경로가 없습니다. 통로 폭, 장애물과 "
                "Nav2 footprint를 확인하세요."
            )
        return path_length(poses)

    def navigate(
        self,
        target: tuple[float, float, float],
        frame_id: str,
        mission_goal: Any,
        *,
        timeout: float,
    ) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(target, frame_id)
        nav_goal = self._send_goal(
            self._navigate_client,
            goal,
            mission_goal,
            10.0,
        )
        with self._active_goal_lock:
            self._active_nav_goal = nav_goal
        # Close the race where STOP arrives after the caller checked the latch
        # but before Nav2 accepted and stored the new goal handle.
        if self._safety_is_paused():
            self.cancel_active_for_safety()
        safety_canceled = False
        try:
            wrapped = self._wait_result(nav_goal, mission_goal, timeout)
        finally:
            with self._active_goal_lock:
                if self._active_nav_goal is nav_goal:
                    self._active_nav_goal = None
                safety_canceled = self._safety_cancel_requested
                self._safety_cancel_requested = False
        if wrapped.status == GoalStatus.STATUS_CANCELED and safety_canceled:
            raise MissionSafetyPaused()
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise MissionCanceled()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionFailure("Nav2 이동에 실패했습니다.")

    def spin(
        self,
        relative_yaw: float,
        mission_goal: Any,
        *,
        timeout: float,
    ) -> None:
        """Run Nav2's collision-checked Spin behavior by a relative angle."""

        goal = Spin.Goal()
        goal.target_yaw = float(relative_yaw)
        goal.time_allowance = Duration(seconds=max(0.0, timeout)).to_msg()
        nav_goal = self._send_goal(
            self._spin_client,
            goal,
            mission_goal,
            10.0,
        )
        with self._active_goal_lock:
            self._active_nav_goal = nav_goal
        if self._safety_is_paused():
            self.cancel_active_for_safety()
        safety_canceled = False
        try:
            wrapped = self._wait_result(nav_goal, mission_goal, timeout)
        finally:
            with self._active_goal_lock:
                if self._active_nav_goal is nav_goal:
                    self._active_nav_goal = None
                safety_canceled = self._safety_cancel_requested
                self._safety_cancel_requested = False
        if wrapped.status == GoalStatus.STATUS_CANCELED and safety_canceled:
            raise MissionSafetyPaused()
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise MissionCanceled()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionFailure(
                "Nav2가 다음 웨이포인트 방향으로 제자리 선회하지 못했습니다. "
                "로봇 주변 장애물과 회전 공간을 확인하세요."
            )

    def cancel_active(self) -> None:
        with self._active_goal_lock:
            nav_goal = self._active_nav_goal
            self._safety_cancel_requested = False
        if nav_goal is not None:
            nav_goal.cancel_goal_async()

    def cancel_active_for_safety(self) -> None:
        """Cancel Nav2 without turning the owning patrol into user cancel."""
        with self._active_goal_lock:
            nav_goal = self._active_nav_goal
            if nav_goal is not None:
                self._safety_cancel_requested = True
        if nav_goal is not None:
            nav_goal.cancel_goal_async()

    def clear_active(self) -> None:
        with self._active_goal_lock:
            self._active_nav_goal = None
            self._safety_cancel_requested = False

    def _pose_stamped(
        self,
        pose: tuple[float, float, float],
        frame_id: str,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = frame_id or "map"
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.orientation.w = math.cos(pose[2] / 2.0)
        return message

    def _send_goal(
        self,
        client: Any,
        goal: Any,
        mission_goal: Any,
        timeout: float,
    ) -> Any:
        future = client.send_goal_async(goal)
        result = self._wait_future(future, mission_goal, timeout)
        if result is None or not result.accepted:
            raise MissionFailure("Nav2가 요청을 수락하지 않았습니다.")
        return result

    def _wait_result(
        self,
        nav_goal: Any,
        mission_goal: Any,
        timeout: float,
    ) -> Any:
        future = nav_goal.get_result_async()
        try:
            return self._wait_future(future, mission_goal, timeout)
        except (MissionCanceled, MissionFailure):
            nav_goal.cancel_goal_async()
            raise

    def _wait_future(
        self,
        future: Any,
        mission_goal: Any,
        timeout: float,
    ) -> Any:
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check_canceled(mission_goal)
            if time.monotonic() >= deadline:
                raise MissionFailure("Nav2 응답 대기 시간이 초과됐습니다.")
            time.sleep(0.05)
        self._check_canceled(mission_goal)
        try:
            return future.result()
        except Exception as exc:
            raise MissionFailure(f"Nav2 요청 처리 오류: {exc}") from exc
