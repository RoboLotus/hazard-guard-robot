from __future__ import annotations

import math
import threading
import time
from typing import Any

import rclpy
from hazard_guard_interfaces.action import RunPatrol
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .errors import MissionCanceled, MissionFailure
from .geometry import pose_errors
from .navigation import Nav2Adapter
from .state import MissionStateStore


class HazardGuardMissionManager(Node):
    """Own ordered patrol execution and delegate movements to Nav2."""

    ACTIVE_STATES = {
        "preparing",
        "running",
        "executing",
        "aligning",
        "dwelling",
    }

    def __init__(self) -> None:
        super().__init__("hazard_guard_mission_manager")
        self.declare_parameter("action_name", "/hazard_guard/run_patrol")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter(
            "compute_path_action_name",
            "/compute_path_to_pose",
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("position_tolerance_m", 0.08)
        self.declare_parameter("yaw_tolerance_rad", 0.05)
        self.declare_parameter("alignment_retries", 2)
        self.declare_parameter("navigation_timeout_sec", 180.0)
        self.declare_parameter("alignment_timeout_sec", 45.0)
        self.declare_parameter("server_wait_timeout_sec", 8.0)

        self._callback_group = ReentrantCallbackGroup()
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            "/hazard_guard/mission/status",
            status_qos,
        )
        self._cancel_service = self.create_service(
            Trigger,
            "/hazard_guard/mission/cancel",
            self._cancel_service_callback,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            RunPatrol,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._state_lock = threading.RLock()
        self._mission_active = False
        self._cancel_requested = threading.Event()
        self._mission_state = MissionStateStore(self._publish_state_payload)
        self._nav = Nav2Adapter(
            self,
            self._callback_group,
            navigate_action_name=str(
                self.get_parameter("navigate_action_name").value
            ),
            compute_path_action_name=str(
                self.get_parameter("compute_path_action_name").value
            ),
            base_frame=str(self.get_parameter("base_frame").value),
            server_wait_timeout_sec=float(
                self.get_parameter("server_wait_timeout_sec").value
            ),
            check_canceled=self._raise_if_canceled,
        )
        self._mission_state.publish()
        self.get_logger().info(
            "Mission manager ready: /hazard_guard/run_patrol -> Nav2"
        )

    def _goal_callback(self, request: RunPatrol.Goal) -> GoalResponse:
        with self._state_lock:
            if self._mission_active:
                self.get_logger().warning(
                    "Rejected patrol: another mission is active"
                )
                return GoalResponse.REJECT
            if not request.waypoints:
                self.get_logger().warning("Rejected patrol: no waypoints")
                return GoalResponse.REJECT
            self._mission_active = True
            self._cancel_requested.clear()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._request_cancel()
        return CancelResponse.ACCEPT

    def _cancel_service_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._state_lock:
            active = self._mission_active
        if not active:
            response.success = False
            response.message = "취소할 활성 순찰 임무가 없습니다."
            return response
        self._request_cancel()
        response.success = True
        response.message = "순찰 중단을 요청했습니다."
        return response

    def _request_cancel(self) -> None:
        self._cancel_requested.set()
        self._nav.cancel_active()
        self._update_state(
            status="canceling",
            accepted=False,
            message="순찰 중단을 요청했습니다.",
        )

    def _execute(self, goal_handle: Any) -> RunPatrol.Result:
        request = goal_handle.request
        waypoints = list(request.waypoints)
        waypoint_states = [
            {
                "id": item.id,
                "name": item.name,
                "x": float(item.x),
                "y": float(item.y),
                "yaw": float(item.yaw),
                "dwell_seconds": float(item.dwell_seconds),
                "enabled": True,
                "status": "pending",
                "message": "대기 중",
            }
            for item in waypoints
        ]
        self._replace_state(
            {
                "mission_id": request.mission_id,
                "name": request.name,
                "status": "preparing",
                "accepted": True,
                "mock": False,
                "frame_id": request.frame_id or "map",
                "current_index": None,
                "total_waypoints": len(waypoints),
                "completed_waypoints": 0,
                "total_distance_m": None,
                "message": "전체 웨이포인트 경로를 확인하고 있습니다.",
                "waypoints": waypoint_states,
            }
        )

        completed = 0
        total_distance = 0.0
        try:
            self._nav.assert_ready()
            start_pose = self._nav.current_pose(request.frame_id)
            if start_pose is None:
                raise MissionFailure(
                    "지도상의 현재 로봇 위치를 확인할 수 없어 순찰을 시작할 수 없습니다."
                )

            segment_start = start_pose
            for index, waypoint in enumerate(waypoints):
                self._raise_if_canceled(goal_handle)
                self._update_waypoint(index, "validating", "Nav2 경로 확인 중")
                self._update_state(
                    status="preparing",
                    current_index=index,
                    message=f"{waypoint.name}까지 이동 가능한 경로를 확인하고 있습니다.",
                )
                target = (
                    float(waypoint.x),
                    float(waypoint.y),
                    float(waypoint.yaw),
                )
                distance = self._nav.compute_path_distance(
                    segment_start,
                    target,
                    request.frame_id,
                    goal_handle,
                )
                total_distance += distance
                segment_start = target
                self._update_waypoint(
                    index,
                    "pending",
                    f"경로 확인 완료 · {distance:.2f}m",
                )

            if request.return_to_start:
                return_distance = self._nav.compute_path_distance(
                    segment_start,
                    start_pose,
                    request.frame_id,
                    goal_handle,
                )
                total_distance += return_distance

            self._update_state(
                status="running",
                current_index=0,
                total_distance_m=round(total_distance, 3),
                message="전체 경로 확인이 완료되어 순찰을 시작합니다.",
            )

            for index, waypoint in enumerate(waypoints):
                self._raise_if_canceled(goal_handle)
                self._update_waypoint(index, "active", "이동 중")
                self._update_state(
                    status="executing",
                    current_index=index,
                    message=f"{waypoint.name}로 이동 중입니다.",
                )
                self._publish_feedback(
                    goal_handle,
                    index=index,
                    waypoint_id=waypoint.id,
                    waypoint_status="active",
                )
                target = (
                    float(waypoint.x),
                    float(waypoint.y),
                    float(waypoint.yaw),
                )
                self._nav.navigate(
                    target,
                    request.frame_id,
                    goal_handle,
                    timeout=float(
                        self.get_parameter("navigation_timeout_sec").value
                    ),
                )
                position_error, yaw_error = self._align(
                    index,
                    waypoint,
                    request.frame_id,
                    goal_handle,
                )

                dwell_seconds = max(0.0, float(waypoint.dwell_seconds))
                if dwell_seconds:
                    self._update_waypoint(
                        index,
                        "dwelling",
                        f"{dwell_seconds:g}초 점검 대기",
                    )
                    self._update_state(
                        status="dwelling",
                        message=f"{waypoint.name}에서 점검 중입니다.",
                    )
                    self._publish_feedback(
                        goal_handle,
                        index=index,
                        waypoint_id=waypoint.id,
                        waypoint_status="dwelling",
                        position_error=position_error,
                        yaw_error=math.degrees(yaw_error),
                    )
                    deadline = time.monotonic() + dwell_seconds
                    while time.monotonic() < deadline:
                        self._raise_if_canceled(goal_handle)
                        remaining = max(0.0, deadline - time.monotonic())
                        time.sleep(min(0.1, remaining))

                completed = index + 1
                self._update_waypoint(index, "completed", "도착 및 점검 완료")
                self._update_state(
                    status="running",
                    completed_waypoints=completed,
                    message=f"{waypoint.name} 점검을 완료했습니다.",
                )
                self._publish_feedback(
                    goal_handle,
                    index=index,
                    waypoint_id=waypoint.id,
                    waypoint_status="completed",
                    position_error=position_error,
                    yaw_error=math.degrees(yaw_error),
                )

            if request.return_to_start:
                self._update_state(
                    status="executing",
                    current_index=None,
                    message="순찰 시작 위치로 복귀 중입니다.",
                )
                self._nav.navigate(
                    start_pose,
                    request.frame_id,
                    goal_handle,
                    timeout=float(
                        self.get_parameter("navigation_timeout_sec").value
                    ),
                )

            goal_handle.succeed()
            self._update_state(
                status="completed",
                accepted=True,
                current_index=None,
                completed_waypoints=completed,
                message="모든 웨이포인트 순찰을 완료했습니다.",
            )
            return self._result(
                True,
                "completed",
                "모든 웨이포인트 순찰을 완료했습니다.",
                completed,
                total_distance,
            )
        except MissionCanceled:
            goal_handle.canceled()
            self._update_state(
                status="canceled",
                accepted=False,
                message="사용자가 순찰을 취소했습니다.",
            )
            return self._result(
                False,
                "canceled",
                "사용자가 순찰을 취소했습니다.",
                completed,
                total_distance,
            )
        except MissionFailure as exc:
            goal_handle.abort()
            current_index = self._mission_state.snapshot().get("current_index")
            if isinstance(current_index, int):
                self._update_waypoint(current_index, "failed", str(exc))
            self._update_state(
                status="failed",
                accepted=False,
                message=f"{exc} 시뮬레이터는 계속 실행됩니다.",
            )
            return self._result(
                False,
                "failed",
                str(exc),
                completed,
                total_distance,
            )
        except Exception as exc:
            goal_handle.abort()
            self.get_logger().error(f"Unexpected mission error: {exc}")
            self._update_state(
                status="failed",
                accepted=False,
                message=f"순찰 처리 오류: {exc}. 시뮬레이터는 계속 실행됩니다.",
            )
            return self._result(
                False,
                "failed",
                f"순찰 처리 오류: {exc}",
                completed,
                total_distance,
            )
        finally:
            with self._state_lock:
                self._mission_active = False
            self._nav.clear_active()
            self._cancel_requested.clear()

    def _align(
        self,
        index: int,
        waypoint: Any,
        frame_id: str,
        mission_goal: Any,
    ) -> tuple[float, float]:
        target = (float(waypoint.x), float(waypoint.y), float(waypoint.yaw))
        retries = max(0, int(self.get_parameter("alignment_retries").value))
        position_tolerance = float(
            self.get_parameter("position_tolerance_m").value
        )
        yaw_tolerance = float(self.get_parameter("yaw_tolerance_rad").value)

        for attempt in range(retries + 1):
            self._raise_if_canceled(mission_goal)
            time.sleep(0.35)
            actual = self._nav.current_pose(frame_id)
            if actual is None:
                raise MissionFailure("최종 로봇 위치와 방향을 확인할 수 없습니다.")
            position_error, yaw_error = pose_errors(actual, target)
            if (
                position_error <= position_tolerance
                and yaw_error <= yaw_tolerance
            ):
                self._update_waypoint(
                    index,
                    "aligned",
                    (
                        f"정렬 완료 · 위치 오차 {position_error:.2f}m "
                        f"· 방향 오차 {math.degrees(yaw_error):.1f}°"
                    ),
                    position_error_m=round(position_error, 3),
                    yaw_error_deg=round(math.degrees(yaw_error), 2),
                    actual_yaw_deg=round(math.degrees(actual[2]), 2),
                )
                return position_error, yaw_error
            if attempt >= retries:
                raise MissionFailure(
                    "최종 정렬 오차가 허용 범위를 벗어났습니다. "
                    f"위치 {position_error:.2f}m, "
                    f"방향 {math.degrees(yaw_error):.1f}°"
                )
            self._update_waypoint(
                index,
                "aligning",
                f"카메라 방향 정렬 중 ({attempt + 1}/{retries})",
                position_error_m=round(position_error, 3),
                yaw_error_deg=round(math.degrees(yaw_error), 2),
                actual_yaw_deg=round(math.degrees(actual[2]), 2),
            )
            self._update_state(
                status="aligning",
                message=f"{waypoint.name}에서 카메라 방향을 정렬하고 있습니다.",
            )
            self._nav.navigate(
                target,
                frame_id,
                mission_goal,
                timeout=float(
                    self.get_parameter("alignment_timeout_sec").value
                ),
            )
        raise MissionFailure("최종 방향 정렬에 실패했습니다.")

    def _raise_if_canceled(self, goal_handle: Any) -> None:
        if self._cancel_requested.is_set() or goal_handle.is_cancel_requested:
            raise MissionCanceled()

    def _replace_state(self, state: dict[str, Any]) -> None:
        self._mission_state.replace(state)

    def _update_state(self, **values: Any) -> None:
        self._mission_state.update(**values)

    def _update_waypoint(
        self,
        index: int,
        status: str,
        message: str,
        **details: Any,
    ) -> None:
        self._mission_state.update_waypoint(
            index,
            status,
            message,
            **details,
        )

    def _publish_state_payload(self, payload: str) -> None:
        message = String()
        message.data = payload
        self._status_publisher.publish(message)

    def _publish_feedback(
        self,
        goal_handle: Any,
        *,
        index: int,
        waypoint_id: str,
        waypoint_status: str,
        position_error: float = -1.0,
        yaw_error: float = -1.0,
    ) -> None:
        feedback = RunPatrol.Feedback()
        state = self._mission_state.snapshot()
        feedback.status = str(state["status"])
        feedback.message = str(state["message"])
        feedback.current_index = index
        feedback.total_waypoints = int(state["total_waypoints"])
        feedback.completed_waypoints = int(state["completed_waypoints"])
        feedback.total_distance_m = float(state["total_distance_m"] or 0.0)
        feedback.waypoint_id = waypoint_id
        feedback.waypoint_status = waypoint_status
        feedback.position_error_m = position_error
        feedback.yaw_error_deg = yaw_error
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        success: bool,
        status: str,
        message: str,
        completed: int,
        total_distance: float,
    ) -> RunPatrol.Result:
        result = RunPatrol.Result()
        result.success = success
        result.status = status
        result.message = message
        result.completed_waypoints = completed
        result.total_distance_m = total_distance
        return result

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HazardGuardMissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
