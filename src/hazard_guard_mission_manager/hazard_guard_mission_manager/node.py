from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from hazard_guard_interfaces.action import RunPatrol
from hazard_guard_interfaces.msg import HazardIncident, PersonSafetyState
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

from .alignment import (
    AlignmentDecision,
    AlignmentThresholds,
    decide_alignment,
    sample_median_pose,
)
from .errors import (
    MissionCanceled,
    MissionFailure,
    MissionSafetyPaused,
    MissionScheduleEnded,
)
from .geometry import (
    departure_rotation,
    forward_approach_pose,
    heading_change_required,
    pose_errors,
)
from .incident import IncidentApprovalLatch, IncidentConflictError
from .incident_detection import thermal_observations
from .navigation import Nav2Adapter
from .schedule import (
    PatrolSchedule,
    REPEAT_COUNT,
    REPEAT_FOREVER,
    REPEAT_ONCE,
    REPEAT_UNTIL_TIME,
    unix_time_ms,
)
from .safety import SafetyPauseLatch
from .state import MissionStateStore


class HazardGuardMissionManager(Node):
    """Own ordered patrol execution and delegate movements to Nav2."""

    ACTIVE_STATES = {
        "preparing",
        "running",
        "executing",
        "aligning",
        "dwelling",
        "scheduled",
        "waiting",
        "approval_required",
        "dispensing",
        "monitoring",
        "admin_release_required",
    }

    def __init__(self) -> None:
        super().__init__("hazard_guard_mission_manager")
        self.declare_parameter("action_name", "/hazard_guard/run_patrol")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("spin_action_name", "/spin")
        self.declare_parameter(
            "compute_path_action_name",
            "/compute_path_to_pose",
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("position_tolerance_m", 0.10)
        self.declare_parameter("yaw_tolerance_rad", 0.10)
        self.declare_parameter("acceptable_position_tolerance_m", 0.15)
        self.declare_parameter("acceptable_yaw_tolerance_rad", 0.17)
        self.declare_parameter("hard_position_tolerance_m", 0.25)
        self.declare_parameter("hard_yaw_tolerance_rad", math.radians(15.0))
        self.declare_parameter("alignment_retries", 1)
        self.declare_parameter("pose_sample_count", 5)
        self.declare_parameter("pose_min_valid_samples", 3)
        self.declare_parameter("pose_sample_interval_sec", 0.15)
        self.declare_parameter("navigation_timeout_sec", 180.0)
        self.declare_parameter("alignment_timeout_sec", 45.0)
        self.declare_parameter("forward_approach_min_distance_m", 0.15)
        self.declare_parameter("pre_rotation_yaw_tolerance_rad", 0.10)
        self.declare_parameter("pre_rotation_timeout_sec", 30.0)
        self.declare_parameter("pre_rotation_retries", 1)
        self.declare_parameter("server_wait_timeout_sec", 8.0)
        self.declare_parameter("safety_supervision_enabled", False)
        self.declare_parameter(
            "person_safety_topic",
            "/hazard_guard/person/safety_state",
        )
        self.declare_parameter(
            "thermal_start_service",
            "/hazard_guard/thermal/start_visit",
        )
        self.declare_parameter(
            "thermal_record_service",
            "/hazard_guard/thermal/record_visit",
        )
        self.declare_parameter("thermal_service_timeout_sec", 5.0)
        self.declare_parameter("hazard_approval_enabled", False)
        self.declare_parameter(
            "hazard_hold_severities", ["warning", "critical"]
        )
        self.declare_parameter(
            "thermal_trend_topic", "/hazard_guard/thermal/trend"
        )
        self.declare_parameter("hazard_evaluation_timeout_sec", 2.0)

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
        self._thermal_inspection_publisher = self.create_publisher(
            String, "/hazard_guard/thermal/inspection_control", 10
        )
        self._incident_publisher = self.create_publisher(
            HazardIncident,
            "/hazard_guard/incidents/status",
            status_qos,
        )
        self._thermal_start_client = self.create_client(
            Trigger,
            str(self.get_parameter("thermal_start_service").value),
            callback_group=self._callback_group,
        )
        self._thermal_record_client = self.create_client(
            Trigger,
            str(self.get_parameter("thermal_record_service").value),
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
        self._active_schedule: PatrolSchedule | None = None
        self._thermal_sequence_faulted = False
        self._thermal_trend_condition = threading.Condition()
        self._thermal_trend_sequence = 0
        self._safety = SafetyPauseLatch(
            enabled=bool(
                self.get_parameter("safety_supervision_enabled").value
            )
        )
        self._incident = IncidentApprovalLatch()
        self._hazard_hold_severities = {
            str(value).lower()
            for value in self.get_parameter("hazard_hold_severities").value
        }
        self._safety_subscription = self.create_subscription(
            PersonSafetyState,
            str(self.get_parameter("person_safety_topic").value),
            self._on_person_safety,
            10,
            callback_group=self._callback_group,
        )
        self._thermal_trend_subscription = self.create_subscription(
            String,
            str(self.get_parameter("thermal_trend_topic").value),
            self._on_thermal_trend,
            10,
            callback_group=self._callback_group,
        )
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
            spin_action_name=str(self.get_parameter("spin_action_name").value),
            base_frame=str(self.get_parameter("base_frame").value),
            server_wait_timeout_sec=float(
                self.get_parameter("server_wait_timeout_sec").value
            ),
            check_canceled=self._raise_if_canceled,
            safety_is_paused=self._operational_is_paused,
        )
        self._mission_state.publish()
        self.get_logger().info(
            "Mission manager ready: /hazard_guard/run_patrol -> Nav2"
        )

    def _on_person_safety(self, message: PersonSafetyState) -> None:
        changed = self._safety.update(message.state, message.reason)
        if not self._safety.is_paused():
            return
        with self._state_lock:
            active = self._mission_active
        if active:
            self._nav.cancel_active_for_safety()
            if changed:
                self._update_state(
                    status="safety_paused",
                    message=(
                        "사람 안전 정지로 순찰을 일시정지했습니다. "
                        f"{message.reason}"
                    ).strip(),
                )

    def _operational_is_paused(self) -> bool:
        return self._safety.is_paused() or self._incident.is_paused()

    def _on_thermal_trend(self, message: String) -> None:
        if not bool(self.get_parameter("hazard_approval_enabled").value):
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("열화상 추세 JSON을 해석하지 못했습니다")
            return
        with self._thermal_trend_condition:
            self._thermal_trend_sequence += 1
            self._thermal_trend_condition.notify_all()
        with self._state_lock:
            active = self._mission_active
        if not active:
            return
        mission_id = self._mission_state.snapshot().get("mission_id")
        observations = thermal_observations(payload, mission_id=mission_id)
        current = self._incident.snapshot()
        if current is not None and current.get("state") in {
            "monitoring",
            "admin_release_required",
        }:
            for observation in observations:
                if observation["equipment_id"] != current.get("equipment_id"):
                    continue
                if observation["severity"] == "normal" and current["state"] == "monitoring":
                    updated = self._incident.mark_monitoring_normalized(
                        "위험 상태가 정상화됐지만 관리자 확인 전까지 순찰을 재개하지 않습니다."
                    )
                    self._publish_incident(updated)
                    self._update_state(
                        status="admin_release_required",
                        message=updated["message"],
                        incident=updated,
                    )
                return
        for observation in observations:
            if observation["severity"] not in self._hazard_hold_severities:
                continue
            try:
                incident, created = self._incident.open(observation)
            except IncidentConflictError as exc:
                self.get_logger().warning(str(exc))
                return
            if not created:
                return
            self._nav.cancel_active_for_safety()
            self._publish_incident(incident)
            self._update_state(
                status="approval_required",
                message="위험 이벤트가 감지되어 관리자 승인을 기다립니다.",
                incident=incident,
            )
            return

    def _publish_incident(self, incident: dict[str, Any]) -> None:
        message = HazardIncident()
        message.stamp = self.get_clock().now().to_msg()
        for name in (
            "incident_id",
            "detection_id",
            "mission_id",
            "equipment_id",
            "source",
            "severity",
            "state",
            "decision",
            "frame_id",
            "message",
        ):
            setattr(message, name, str(incident.get(name) or ""))
        for name in ("x", "y", "z", "temperature_c", "confidence"):
            setattr(message, name, float(incident.get(name) or 0.0))
        message.simulated = bool(incident.get("simulated", False))
        self._incident_publisher.publish(message)

    def _goal_callback(self, request: RunPatrol.Goal) -> GoalResponse:
        with self._state_lock:
            if self._incident.is_paused():
                self.get_logger().warning(
                    "Rejected patrol: hazard approval is still pending"
                )
                return GoalResponse.REJECT
            if self._mission_active:
                self.get_logger().warning(
                    "Rejected patrol: another mission is active"
                )
                return GoalResponse.REJECT
            if not request.waypoints:
                self.get_logger().warning("Rejected patrol: no waypoints")
                return GoalResponse.REJECT
            try:
                schedule = PatrolSchedule.from_request(request)
            except ValueError as exc:
                self.get_logger().warning(f"Rejected patrol schedule: {exc}")
                return GoalResponse.REJECT
            self._mission_active = True
            self._cancel_requested.clear()
            self._active_schedule = schedule
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
        incident = self._incident.cancel("순찰 임무가 취소되었습니다.")
        if incident is not None:
            self._publish_incident(incident)
        self._update_state(
            status="canceling",
            accepted=False,
            message="순찰 중단을 요청했습니다.",
        )

    def _execute(self, goal_handle: Any) -> RunPatrol.Result:
        request = goal_handle.request
        waypoints = list(request.waypoints)
        schedule = self._active_schedule or PatrolSchedule.from_request(request)
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
                "repeat_mode": {
                    REPEAT_ONCE: "once",
                    REPEAT_COUNT: "count",
                    REPEAT_UNTIL_TIME: "until_time",
                    REPEAT_FOREVER: "forever",
                }[schedule.repeat_mode],
                "repeat_count": schedule.repeat_count,
                "repeat_interval_sec": schedule.repeat_interval_sec,
                "current_cycle": 0,
                "total_cycles": schedule.total_cycles,
                "completed_cycles": 0,
                "start_at_unix_ms": schedule.start_at_unix_ms,
                "end_at_unix_ms": schedule.end_at_unix_ms,
                "next_run_at_unix_ms": schedule.start_at_unix_ms,
                "total_distance_m": None,
                "message": "전체 웨이포인트 경로를 확인하고 있습니다.",
                "waypoints": waypoint_states,
            }
        )

        completed = 0
        completed_cycles = 0
        total_distance = 0.0
        self._thermal_sequence_faulted = False
        try:
            self._wait_for_scheduled_start(goal_handle, schedule)
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
                final_target = (
                    float(waypoint.x),
                    float(waypoint.y),
                    float(waypoint.yaw),
                )
                approach_target = self._forward_approach(
                    segment_start,
                    final_target,
                )
                distance = self._nav.compute_path_distance(
                    segment_start,
                    approach_target,
                    request.frame_id,
                    goal_handle,
                )
                total_distance += distance
                segment_start = final_target
                self._update_waypoint(
                    index,
                    "pending",
                    f"경로 확인 완료 · {distance:.2f}m",
                )

            if request.return_to_start:
                return_distance = self._nav.compute_path_distance(
                    segment_start,
                    self._forward_approach(segment_start, start_pose),
                    request.frame_id,
                    goal_handle,
                )
                total_distance += return_distance
            elif schedule.repeat_mode != REPEAT_ONCE:
                first_waypoint = waypoints[0]
                first_target = (
                    float(first_waypoint.x),
                    float(first_waypoint.y),
                    float(first_waypoint.yaw),
                )
                self._nav.compute_path_distance(
                    segment_start,
                    self._forward_approach(segment_start, first_target),
                    request.frame_id,
                    goal_handle,
                )

            self._update_state(
                status="running",
                current_index=0,
                total_distance_m=round(total_distance, 3),
                next_run_at_unix_ms=0,
                message="전체 경로 확인이 완료되어 순찰을 시작합니다.",
            )

            while schedule.should_continue(completed_cycles):
                self._raise_if_canceled(goal_handle)
                current_cycle = completed_cycles + 1
                completed = 0
                for index in range(len(waypoints)):
                    self._update_waypoint(index, "pending", "대기 중")
                self._update_state(
                    status="running",
                    current_cycle=current_cycle,
                    completed_waypoints=0,
                    message=self._cycle_message(current_cycle, schedule),
                )

                self._start_thermal_visit(f"cycle {current_cycle}")

                completed = self._run_cycle(
                    goal_handle,
                    request,
                    waypoints,
                    current_cycle,
                )

                if request.return_to_start:
                    self._update_state(
                        status="executing",
                        current_index=None,
                        message=f"{current_cycle}회차 시작 위치로 복귀 중입니다.",
                    )
                    self._navigate_forward_to_pose(
                        start_pose,
                        request.frame_id,
                        goal_handle,
                        timeout=float(
                            self.get_parameter("navigation_timeout_sec").value
                        ),
                    )

                with self._thermal_trend_condition:
                    previous_trend_sequence = self._thermal_trend_sequence
                recorded = self._record_thermal_visit(f"cycle {current_cycle}")
                if recorded and bool(
                    self.get_parameter("hazard_approval_enabled").value
                ):
                    self._wait_for_thermal_evaluation(
                        goal_handle,
                        previous_trend_sequence,
                    )
                    self._wait_for_safety_clear(goal_handle)

                completed_cycles += 1
                self._update_state(
                    completed_cycles=completed_cycles,
                    current_index=None,
                    message=f"{completed_cycles}회차 순찰을 완료했습니다.",
                )
                if not schedule.should_continue(completed_cycles):
                    break
                self._wait_between_cycles(
                    goal_handle,
                    schedule,
                    completed_cycles,
                )

            goal_handle.succeed()
            self._update_state(
                status="completed",
                accepted=True,
                current_index=None,
                completed_waypoints=completed,
                completed_cycles=completed_cycles,
                next_run_at_unix_ms=0,
                message=f"예약된 순찰 {completed_cycles}회차를 완료했습니다.",
            )
            return self._result(
                True,
                "completed",
                f"예약된 순찰 {completed_cycles}회차를 완료했습니다.",
                completed,
                completed_cycles,
                total_distance,
            )
        except MissionScheduleEnded:
            goal_handle.succeed()
            message = (
                "예약 종료 시각이 되어 순찰을 종료했습니다. "
                f"({completed_cycles}회 완료)"
            )
            self._update_state(
                status="completed",
                accepted=True,
                current_index=None,
                completed_cycles=completed_cycles,
                next_run_at_unix_ms=0,
                message=message,
            )
            return self._result(
                True,
                "completed",
                message,
                completed,
                completed_cycles,
                total_distance,
            )
        except MissionCanceled:
            goal_handle.canceled()
            self._update_state(
                status="canceled",
                accepted=False,
                next_run_at_unix_ms=0,
                message="사용자가 순찰을 취소했습니다.",
            )
            return self._result(
                False,
                "canceled",
                "사용자가 순찰을 취소했습니다.",
                completed,
                completed_cycles,
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
                next_run_at_unix_ms=0,
                message=f"{exc} 시뮬레이터는 계속 실행됩니다.",
            )
            return self._result(
                False,
                "failed",
                str(exc),
                completed,
                completed_cycles,
                total_distance,
            )
        except Exception as exc:
            goal_handle.abort()
            self.get_logger().error(f"Unexpected mission error: {exc}")
            self._update_state(
                status="failed",
                accepted=False,
                next_run_at_unix_ms=0,
                message=f"순찰 처리 오류: {exc}. 시뮬레이터는 계속 실행됩니다.",
            )
            return self._result(
                False,
                "failed",
                f"순찰 처리 오류: {exc}",
                completed,
                completed_cycles,
                total_distance,
            )
        finally:
            with self._state_lock:
                self._mission_active = False
                self._active_schedule = None
            self._nav.clear_active()
            self._cancel_requested.clear()

    @staticmethod
    def _thermal_equipment_id(waypoint: Any) -> str | None:
        equipment_id = str(getattr(waypoint, "equipment_id", "")).strip()
        return equipment_id or None

    def _set_thermal_focus(self, equipment_id: str | None) -> None:
        message = String()
        payload = {"action": "focus_equipment", "equipment_id": equipment_id} if equipment_id else {"action": "clear_focus"}
        message.data = json.dumps(payload, separators=(",", ":"))
        self._thermal_inspection_publisher.publish(message)

    def _call_thermal_service(self, client: Any, label: str) -> bool:
        """Complete one thermal visit transition before the next can start."""

        if self._thermal_sequence_faulted:
            return False
        if not client.service_is_ready():
            self.get_logger().info(f"{label}: thermal service is not active")
            return False

        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        timeout = max(
            0.1,
            float(self.get_parameter("thermal_service_timeout_sec").value),
        )
        if not completed.wait(timeout):
            self._thermal_sequence_faulted = True
            future.cancel()
            self.get_logger().error(
                f"{label}: thermal service timed out after {timeout:g}s; "
                "thermal visit sequencing is disabled for this mission"
            )
            return False
        try:
            response = future.result()
        except Exception as exc:
            self._thermal_sequence_faulted = True
            self.get_logger().warning(f"{label}: thermal service failed: {exc}")
            return False
        if not response.success:
            self.get_logger().warning(f"{label}: {response.message}")
            return False
        self.get_logger().info(f"{label}: {response.message}")
        return True

    def _start_thermal_visit(self, cycle_name: str) -> bool:
        """Reset the thermal accumulator once before a patrol cycle."""

        return self._call_thermal_service(
            self._thermal_start_client,
            f"{cycle_name}: thermal visit start",
        )

    def _record_thermal_visit(self, cycle_name: str) -> bool:
        """Persist a completed visit before another patrol cycle can start."""

        return self._call_thermal_service(
            self._thermal_record_client,
            f"{cycle_name}: thermal visit record",
        )

    def _run_cycle(
        self,
        goal_handle: Any,
        request: RunPatrol.Goal,
        waypoints: list[Any],
        current_cycle: int,
    ) -> int:
        completed = 0
        for index, waypoint in enumerate(waypoints):
            self._raise_if_canceled(goal_handle)
            self._update_waypoint(index, "active", "이동 중")
            self._update_state(
                status="executing",
                current_index=index,
                message=f"{current_cycle}회차 · {waypoint.name}로 이동 중입니다.",
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
            self._navigate_forward_to_pose(
                target,
                request.frame_id,
                goal_handle,
                timeout=float(
                    self.get_parameter("navigation_timeout_sec").value
                ),
                waypoint_index=index,
                waypoint_name=str(waypoint.name),
            )
            position_error, yaw_error = self._align(
                index,
                waypoint,
                request.frame_id,
                goal_handle,
            )

            dwell_seconds = max(0.0, float(waypoint.dwell_seconds))
            if dwell_seconds:
                self._set_thermal_focus(self._thermal_equipment_id(waypoint))
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
                remaining = dwell_seconds
                while remaining > 0.0:
                    self._raise_if_canceled(goal_handle)
                    self._wait_for_safety_clear(goal_handle)
                    slice_seconds = min(0.1, remaining)
                    started = time.monotonic()
                    time.sleep(slice_seconds)
                    if not self._operational_is_paused():
                        remaining -= time.monotonic() - started
                self._set_thermal_focus(None)
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
        return completed

    def _forward_approach(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return forward_approach_pose(
            current,
            target,
            minimum_distance_m=float(
                self.get_parameter("forward_approach_min_distance_m").value
            ),
        )

    def _navigate_forward_to_pose(
        self,
        target: tuple[float, float, float],
        frame_id: str,
        goal_handle: Any,
        *,
        timeout: float,
        waypoint_index: int | None = None,
        waypoint_name: str = "시작 위치",
    ) -> None:
        """Drive toward the target first, then rotate to inspection heading."""

        current = self._nav.current_pose(frame_id)
        if current is None:
            raise MissionFailure(
                "전진 접근 방향을 계산할 현재 로봇 위치를 확인할 수 없습니다."
            )
        approach = self._forward_approach(current, target)
        approach_yaw_deg = round(math.degrees(approach[2]), 2)
        relative_yaw = departure_rotation(
            current,
            target,
            minimum_distance_m=float(
                self.get_parameter("forward_approach_min_distance_m").value
            ),
            tolerance_rad=float(
                self.get_parameter("pre_rotation_yaw_tolerance_rad").value
            ),
        )
        if relative_yaw is not None:
            if waypoint_index is not None:
                self._update_waypoint(
                    waypoint_index,
                    "aligning",
                    "출발 전 다음 웨이포인트 방향으로 제자리 선회 중",
                    approach_yaw_deg=approach_yaw_deg,
                    pre_rotation_deg=round(math.degrees(relative_yaw), 2),
                )
            self._update_state(
                status="aligning",
                message=(
                    f"{waypoint_name} 방향으로 먼저 제자리 선회하고 있습니다. "
                    f"(진행 방향 {approach_yaw_deg:.1f}°)"
                ),
            )
            self._spin_to_heading_with_safety_retry(
                approach[2],
                frame_id,
                goal_handle,
            )
        if waypoint_index is not None:
            self._update_waypoint(
                waypoint_index,
                "active",
                "다음 지점을 바라보며 전진 접근 중",
                approach_yaw_deg=approach_yaw_deg,
            )
        self._update_state(
            status="executing",
            message=(
                f"{waypoint_name}까지 전진 방향으로 이동하고 있습니다. "
                f"(접근 방향 {approach_yaw_deg:.1f}°)"
            ),
        )
        self._navigate_with_safety_retry(
            approach,
            frame_id,
            goal_handle,
            timeout=timeout,
        )

        if not heading_change_required(
            approach,
            target,
            tolerance_rad=float(self.get_parameter("yaw_tolerance_rad").value),
        ):
            return

        if waypoint_index is not None:
            self._update_waypoint(
                waypoint_index,
                "aligning",
                "도착 후 검사 방향 정렬 중",
                approach_yaw_deg=approach_yaw_deg,
                inspection_yaw_deg=round(math.degrees(target[2]), 2),
            )
        self._update_state(
            status="aligning",
            message=f"{waypoint_name}에 도착해 검사 방향으로 회전하고 있습니다.",
        )
        self._navigate_with_safety_retry(
            target,
            frame_id,
            goal_handle,
            timeout=float(self.get_parameter("alignment_timeout_sec").value),
        )

    def _spin_to_heading_with_safety_retry(
        self,
        target_yaw: float,
        frame_id: str,
        goal_handle: Any,
    ) -> None:
        """Face an absolute map heading before translation.

        Nav2 Spin accepts a relative angle. Recompute that angle from the
        latest TF for every safety resume and correction attempt so a partial
        turn is never applied twice.
        """

        tolerance = max(
            0.0,
            float(self.get_parameter("pre_rotation_yaw_tolerance_rad").value),
        )
        timeout = max(
            0.1,
            float(self.get_parameter("pre_rotation_timeout_sec").value),
        )
        retries = max(0, int(self.get_parameter("pre_rotation_retries").value))
        attempt = 0
        while attempt <= retries:
            self._raise_if_canceled(goal_handle)
            self._wait_for_safety_clear(goal_handle)
            current = self._nav.current_pose(frame_id)
            if current is None:
                raise MissionFailure(
                    "제자리 선회에 사용할 현재 로봇 방향을 확인할 수 없습니다."
                )
            remaining = math.atan2(
                math.sin(target_yaw - current[2]),
                math.cos(target_yaw - current[2]),
            )
            if abs(remaining) <= tolerance:
                return
            try:
                self._nav.spin(
                    remaining,
                    goal_handle,
                    timeout=timeout,
                )
            except MissionSafetyPaused:
                continue
            time.sleep(0.1)
            actual = self._nav.current_pose(frame_id)
            if actual is not None:
                residual = math.atan2(
                    math.sin(target_yaw - actual[2]),
                    math.cos(target_yaw - actual[2]),
                )
                if abs(residual) <= tolerance:
                    return
            attempt += 1
        raise MissionFailure(
            "다음 웨이포인트 진행 방향으로 제자리 선회한 뒤에도 "
            "방향 오차가 허용 범위를 벗어났습니다."
        )

    def _navigate_with_safety_retry(
        self,
        target: tuple[float, float, float],
        frame_id: str,
        goal_handle: Any,
        *,
        timeout: float,
    ) -> None:
        """Retry the same Nav2 target after a person-safety pause clears."""
        while True:
            self._raise_if_canceled(goal_handle)
            self._wait_for_safety_clear(goal_handle)
            self._raise_if_canceled(goal_handle)
            try:
                self._nav.navigate(
                    target,
                    frame_id,
                    goal_handle,
                    timeout=timeout,
                )
                return
            except MissionSafetyPaused:
                # The safety callback canceled Nav2 deliberately. Keep the
                # mission and waypoint intact, wait for clear, then replan.
                continue

    def _wait_for_safety_clear(self, goal_handle: Any) -> None:
        announced = False
        while self._operational_is_paused():
            self._raise_if_canceled(goal_handle)
            if not announced:
                incident = self._incident.snapshot()
                if incident is not None and self._incident.is_paused():
                    self._update_state(
                        status=str(incident["state"]),
                        message=str(
                            incident.get("message")
                            or "위험 이벤트 관리자 승인을 기다립니다."
                        ),
                        incident=incident,
                    )
                else:
                    _state, reason = self._safety.snapshot()
                    self._update_state(
                        status="safety_paused",
                        message=(
                            "사람 안전 구역이 확보될 때까지 대기합니다. "
                            f"{reason}"
                        ).strip(),
                    )
                announced = True
            time.sleep(0.05)
        if announced:
            self._update_state(
                status="executing",
                message="안전 구역이 확보되어 현재 목적지를 다시 계획합니다.",
            )

    def _wait_for_thermal_evaluation(
        self,
        goal_handle: Any,
        previous_sequence: int,
    ) -> None:
        timeout = float(
            self.get_parameter("hazard_evaluation_timeout_sec").value
        )
        deadline = time.monotonic() + max(0.0, timeout)
        with self._thermal_trend_condition:
            while self._thermal_trend_sequence <= previous_sequence:
                self._raise_if_canceled(goal_handle)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.get_logger().warning(
                        "열화상 위험 판정 결과 대기 시간이 초과됐습니다"
                    )
                    return
                self._thermal_trend_condition.wait(
                    timeout=min(0.1, remaining)
                )

    def _wait_for_scheduled_start(
        self,
        goal_handle: Any,
        schedule: PatrolSchedule,
    ) -> None:
        if schedule.start_at_unix_ms <= 0:
            return
        self._wait_until(
            goal_handle,
            schedule.start_at_unix_ms,
            status="scheduled",
            message="예약 시작 시각까지 대기하고 있습니다.",
        )

    def _wait_between_cycles(
        self,
        goal_handle: Any,
        schedule: PatrolSchedule,
        completed_cycles: int,
    ) -> None:
        if schedule.repeat_interval_sec <= 0:
            return
        next_run = unix_time_ms() + int(schedule.repeat_interval_sec * 1000)
        self._wait_until(
            goal_handle,
            next_run,
            status="waiting",
            message=f"{completed_cycles}회 완료 · 다음 순찰까지 대기 중입니다.",
        )

    def _wait_until(
        self,
        goal_handle: Any,
        target_unix_ms: int,
        *,
        status: str,
        message: str,
    ) -> None:
        last_remaining = None
        while True:
            self._raise_if_canceled(goal_handle)
            remaining_ms = target_unix_ms - unix_time_ms()
            if remaining_ms <= 0:
                break
            remaining_sec = max(1, math.ceil(remaining_ms / 1000))
            if remaining_sec != last_remaining:
                self._update_state(
                    status=status,
                    current_index=None,
                    next_run_at_unix_ms=target_unix_ms,
                    message=f"{message} ({remaining_sec}초 남음)",
                )
                last_remaining = remaining_sec
            time.sleep(min(0.25, remaining_ms / 1000))
        self._update_state(next_run_at_unix_ms=0)

    @staticmethod
    def _cycle_message(current_cycle: int, schedule: PatrolSchedule) -> str:
        if schedule.total_cycles:
            return f"순찰 {current_cycle}/{schedule.total_cycles}회차를 시작합니다."
        return f"순찰 {current_cycle}회차를 시작합니다."

    def _align(
        self,
        index: int,
        waypoint: Any,
        frame_id: str,
        mission_goal: Any,
    ) -> tuple[float, float]:
        self._wait_for_safety_clear(mission_goal)
        target = (float(waypoint.x), float(waypoint.y), float(waypoint.yaw))
        retries = max(0, int(self.get_parameter("alignment_retries").value))
        thresholds = AlignmentThresholds(
            normal_position_m=float(
                self.get_parameter("position_tolerance_m").value
            ),
            normal_yaw_rad=float(
                self.get_parameter("yaw_tolerance_rad").value
            ),
            acceptable_position_m=float(
                self.get_parameter("acceptable_position_tolerance_m").value
            ),
            acceptable_yaw_rad=float(
                self.get_parameter("acceptable_yaw_tolerance_rad").value
            ),
            hard_position_m=float(
                self.get_parameter("hard_position_tolerance_m").value
            ),
            hard_yaw_rad=float(
                self.get_parameter("hard_yaw_tolerance_rad").value
            ),
        )
        sample_count = int(self.get_parameter("pose_sample_count").value)
        min_valid_samples = int(
            self.get_parameter("pose_min_valid_samples").value
        )
        sample_interval = float(
            self.get_parameter("pose_sample_interval_sec").value
        )

        for attempt in range(retries + 1):
            self._raise_if_canceled(mission_goal)
            self._wait_for_safety_clear(mission_goal)
            actual = sample_median_pose(
                lambda: self._nav.current_pose(frame_id),
                sample_count=sample_count,
                min_valid_samples=min_valid_samples,
                interval_sec=sample_interval,
            )
            if actual is None:
                raise MissionFailure("최종 로봇 위치와 방향을 확인할 수 없습니다.")
            position_error, yaw_error = pose_errors(actual, target)
            decision = decide_alignment(
                position_error,
                yaw_error,
                thresholds,
                attempt=attempt,
                retries=retries,
            )
            if decision in (
                AlignmentDecision.ALIGNED,
                AlignmentDecision.ACCEPTED,
            ):
                quality = (
                    "normal"
                    if decision == AlignmentDecision.ALIGNED
                    else "acceptable"
                )
                prefix = "정렬 완료" if quality == "normal" else "허용 오차로 계속"
                if quality == "acceptable":
                    self.get_logger().warning(
                        f"Accepted waypoint alignment: {waypoint.name}, "
                        f"position={position_error:.3f}m, "
                        f"yaw={math.degrees(yaw_error):.2f}deg"
                    )
                self._update_waypoint(
                    index,
                    "aligned",
                    (
                        f"{prefix} · 위치 오차 {position_error:.2f}m "
                        f"· 방향 오차 {math.degrees(yaw_error):.1f}°"
                    ),
                    position_error_m=round(position_error, 3),
                    yaw_error_deg=round(math.degrees(yaw_error), 2),
                    actual_yaw_deg=round(math.degrees(actual[2]), 2),
                    alignment_quality=quality,
                )
                return position_error, yaw_error
            if decision in (
                AlignmentDecision.FAILED,
                AlignmentDecision.FAILED_HARD,
            ):
                severity = (
                    "안전 한계"
                    if decision == AlignmentDecision.FAILED_HARD
                    else "허용 범위"
                )
                raise MissionFailure(
                    f"최종 정렬 오차가 {severity}를 벗어났습니다. "
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
            self._navigate_with_safety_retry(
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
        schedule = self._active_schedule
        if schedule is not None and schedule.deadline_reached():
            raise MissionScheduleEnded()

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
        feedback.current_cycle = int(state["current_cycle"])
        feedback.total_cycles = int(state["total_cycles"])
        feedback.completed_cycles = int(state["completed_cycles"])
        feedback.next_run_at_unix_ms = int(state["next_run_at_unix_ms"])
        feedback.end_at_unix_ms = int(state["end_at_unix_ms"])
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
        completed_cycles: int,
        total_distance: float,
    ) -> RunPatrol.Result:
        result = RunPatrol.Result()
        result.success = success
        result.status = status
        result.message = message
        result.completed_waypoints = completed
        result.completed_cycles = completed_cycles
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
