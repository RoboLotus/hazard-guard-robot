from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from typing import Any

import rclpy
from hazard_guard_interfaces.action import DispenseBeacon, RunPatrol
from hazard_guard_interfaces.msg import HazardIncident, PersonSafetyState
from hazard_guard_interfaces.srv import (
    DispenserRequestStatus,
    HazardDecision,
    RecordThermalVisit,
)
from rclpy.action import (
    ActionClient,
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
from .incident import (
    DECISION_ACKNOWLEDGE_FIELD_CHECK,
    DECISION_COMPLETE_MONITORING,
    DECISION_DROP_THEN_MONITOR,
    DECISION_DROP_THEN_RESUME,
    DECISION_RESUME,
    PROGRESS_DISPENSER_STATES,
    TERMINAL_DISPENSER_STATES,
)
from .incident_detection import thermal_observations, thermal_visit_index
from .dispenser_auth import (
    command_authorization,
    valid_decision_authorization,
)
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
        self.declare_parameter("yaw_tolerance_rad", math.radians(5.0))
        self.declare_parameter("acceptable_position_tolerance_m", 0.15)
        self.declare_parameter(
            "acceptable_yaw_tolerance_rad", math.radians(5.0)
        )
        self.declare_parameter("hard_position_tolerance_m", 0.25)
        self.declare_parameter("hard_yaw_tolerance_rad", math.radians(15.0))
        self.declare_parameter("alignment_retries", 1)
        self.declare_parameter("pose_sample_count", 5)
        self.declare_parameter("pose_min_valid_samples", 3)
        self.declare_parameter("pose_sample_interval_sec", 0.15)
        self.declare_parameter("navigation_timeout_sec", 180.0)
        self.declare_parameter("alignment_timeout_sec", 45.0)
        self.declare_parameter("forward_approach_min_distance_m", 0.15)
        self.declare_parameter(
            "pre_rotation_yaw_tolerance_rad", math.radians(5.0)
        )
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
            "/hazard_guard/thermal/record_visit_correlated",
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
        self.declare_parameter("monitoring_interval_sec", 10.0)
        self.declare_parameter("monitoring_dwell_sec", 3.0)
        self.declare_parameter("monitoring_result_timeout_sec", 8.0)
        self.declare_parameter("monitoring_max_failures", 3)
        self.declare_parameter("dispenser_rear_offset_m", -0.25)
        self.declare_parameter(
            "dispenser_action_name", "/hazard_guard/dispenser/dispense"
        )
        self.declare_parameter(
            "dispenser_result_topic", "/hazard_guard/dispenser/result"
        )
        self.declare_parameter("dispenser_verification_timeout_sec", 30.0)

        self._dispenser_approval_secret = os.getenv(
            "HAZARD_GUARD_DISPENSER_APPROVAL_SECRET", ""
        ).strip()
        if (
            bool(self.get_parameter("hazard_approval_enabled").value)
            and not self._dispenser_approval_secret
        ):
            raise ValueError(
                "위험 승인 흐름에는 HAZARD_GUARD_DISPENSER_APPROVAL_SECRET가 필요합니다"
            )

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
        self._dispenser_action_client = ActionClient(
            self,
            DispenseBeacon,
            str(self.get_parameter("dispenser_action_name").value),
            callback_group=self._callback_group,
        )
        self._dispenser_result_subscription = self.create_subscription(
            String,
            str(self.get_parameter("dispenser_result_topic").value),
            self._on_dispenser_result,
            10,
            callback_group=self._callback_group,
        )
        self._dispenser_request_status_client = self.create_client(
            DispenserRequestStatus,
            "/hazard_guard/dispenser/request_status",
            callback_group=self._callback_group,
        )
        self._incident_decision_service = self.create_service(
            HazardDecision,
            "/hazard_guard/incidents/decision",
            self._on_incident_decision,
            callback_group=self._callback_group,
        )
        self._thermal_start_client = self.create_client(
            Trigger,
            str(self.get_parameter("thermal_start_service").value),
            callback_group=self._callback_group,
        )
        self._thermal_record_client = self.create_client(
            RecordThermalVisit,
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
        self._expected_thermal_correlations: set[str] = set()
        self._thermal_completed_correlations: set[str] = set()
        self._pending_dispenser_verifications: dict[str, dict[str, Any]] = {}
        self._monitoring_lock = threading.Lock()
        self._monitoring_phase = "idle"
        self._monitoring_due_at = 0.0
        self._monitoring_pending_correlation: str | None = None
        self._monitoring_pending_deadline = 0.0
        self._monitoring_failures = 0
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
        self._dispenser_verification_timer = self.create_timer(
            0.5,
            self._poll_pending_dispenser_results,
            callback_group=self._callback_group,
        )
        self._monitoring_timer = self.create_timer(
            0.25,
            self._monitoring_tick,
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
        if not isinstance(payload, dict):
            self.get_logger().warning("열화상 추세 결과가 JSON 객체가 아닙니다")
            return
        with self._state_lock:
            if not self._mission_active:
                return
            mission_id = self._mission_state.snapshot().get("mission_id")
            correlation_id = str(payload.get("correlation_id") or "").strip()
            with self._thermal_trend_condition:
                expected = correlation_id in self._expected_thermal_correlations
            if not expected:
                self.get_logger().warning(
                    "현재 순찰 기록과 일치하지 않는 열화상 결과를 무시했습니다"
                )
                return
            try:
                thermal_visit_index(payload)
                observations = thermal_observations(
                    payload,
                    mission_id=mission_id,
                )
            except ValueError as exc:
                self.get_logger().warning(f"열화상 추세 결과 거부: {exc}")
                return
            if not observations:
                self.get_logger().warning(
                    "열화상 추세 결과에 유효한 설비 관측이 없습니다"
                )
                return
            with self._thermal_trend_condition:
                self._thermal_completed_correlations.add(correlation_id)
                self._thermal_trend_condition.notify_all()
            with self._monitoring_lock:
                monitoring_result = (
                    correlation_id == self._monitoring_pending_correlation
                )
                if monitoring_result:
                    self._monitoring_pending_correlation = None
                    self._monitoring_pending_deadline = 0.0
                    self._monitoring_due_at = (
                        time.monotonic()
                        + max(
                            0.5,
                            float(
                                self.get_parameter(
                                    "monitoring_interval_sec"
                                ).value
                            ),
                        )
                    )
            current = self._incident.snapshot()
            if current is not None and current.get("state") in {
                "monitoring",
                "admin_release_required",
            }:
                for observation in observations:
                    if observation["equipment_id"] != current.get("equipment_id"):
                        continue
                    if (
                        observation["severity"] == "normal"
                        and current["state"] == "monitoring"
                    ):
                        try:
                            updated = self._incident.mark_monitoring_normalized(
                                "위험 상태가 정상화됐지만 관리자 확인 전까지 순찰을 재개하지 않습니다."
                            )
                        except IncidentConflictError:
                            # A concurrent administrator decision already moved
                            # the incident forward; this sensor result is stale.
                            return
                        self._publish_incident(updated)
                        self._update_state(
                            status="admin_release_required",
                            message=updated["message"],
                            incident=updated,
                        )
                        self._reset_monitoring_state(clear_focus=True)
                    if monitoring_result:
                        with self._thermal_trend_condition:
                            self._expected_thermal_correlations.discard(
                                correlation_id
                            )
                            self._thermal_completed_correlations.discard(
                                correlation_id
                            )
                    return
                if monitoring_result:
                    with self._thermal_trend_condition:
                        self._expected_thermal_correlations.discard(
                            correlation_id
                        )
                        self._thermal_completed_correlations.discard(
                            correlation_id
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
        message.beacon_pose_available = bool(
            incident.get("beacon_pose_available", False)
        )
        message.beacon_frame_id = str(incident.get("beacon_frame_id") or "")
        for name in ("beacon_x", "beacon_y", "beacon_z", "beacon_yaw"):
            setattr(message, name, float(incident.get(name) or 0.0))
        self._incident_publisher.publish(message)

    @staticmethod
    def _decision_response(
        response: HazardDecision.Response,
        *,
        accepted: bool,
        record: dict[str, Any] | None,
        request_id: str,
        decision: str,
        message: str,
    ) -> HazardDecision.Response:
        response.accepted = bool(accepted)
        response.incident_id = str((record or {}).get("incident_id") or "")
        response.request_id = str(request_id)
        response.decision = str(decision)
        response.state = str((record or {}).get("state") or "unavailable")
        response.message = str(message)
        return response

    def _on_incident_decision(
        self,
        request: HazardDecision.Request,
        response: HazardDecision.Response,
    ) -> HazardDecision.Response:
        incident_id = str(request.incident_id).strip()
        decision = str(request.decision).strip()
        request_id = str(request.request_id).strip()
        operator_id = str(request.operator_id).strip()
        with self._state_lock:
            try:
                if not valid_decision_authorization(
                    self._dispenser_approval_secret,
                    incident_id=incident_id,
                    request_id=request_id,
                    decision=decision,
                    operator_id=operator_id,
                    authorization=str(request.authorization),
                ):
                    raise IncidentConflictError(
                        "관리자 결정 승인값이 없거나 올바르지 않습니다"
                    )
                if decision in {
                    DECISION_DROP_THEN_RESUME,
                    DECISION_DROP_THEN_MONITOR,
                } and self._safety.is_paused():
                    raise IncidentConflictError(
                        "사람 안전 상태가 CLEAR가 아니므로 비콘을 배출할 수 없습니다"
                    )
                if (
                    decision
                    in {DECISION_DROP_THEN_RESUME, DECISION_DROP_THEN_MONITOR}
                    and self._mission_active
                    and self._cancel_requested.is_set()
                ):
                    raise IncidentConflictError(
                        "순찰 취소 처리 중입니다. 취소 완료 후 배출을 다시 확인해 주세요"
                    )
                record, created = self._incident.decide(
                    incident_id=incident_id,
                    request_id=request_id,
                    decision=decision,
                    operator_id=operator_id,
                )
            except (ValueError, IncidentConflictError) as exc:
                return self._decision_response(
                    response,
                    accepted=False,
                    record=self._incident.snapshot(),
                    request_id=request_id,
                    decision=decision,
                    message=str(exc),
                )

            if not created:
                current = self._incident.snapshot()
                can_republish = (
                    decision
                    in {DECISION_DROP_THEN_RESUME, DECISION_DROP_THEN_MONITOR}
                    and current is not None
                    and current.get("incident_id") == incident_id
                    and current.get("request_id") == request_id
                    and current.get("decision") == decision
                    and current.get("state") == "dispensing"
                )
                if can_republish:
                    self._publish_dispenser_drop(current, request_id)
                return self._decision_response(
                    response,
                    accepted=True,
                    record=current or record,
                    request_id=request_id,
                    decision=decision,
                    message="이미 처리된 관리자 결정입니다.",
                )

            if decision in {
                DECISION_RESUME,
                DECISION_COMPLETE_MONITORING,
                DECISION_ACKNOWLEDGE_FIELD_CHECK,
            }:
                try:
                    record = self._incident.resolve_resume()
                except IncidentConflictError as exc:
                    return self._decision_response(
                        response,
                        accepted=False,
                        record=self._incident.snapshot(),
                        request_id=request_id,
                        decision=decision,
                        message=str(exc),
                    )
                mission_active = self._mission_active
                mission_message = (
                    "관리자 확인에 따라 순찰을 재개합니다."
                    if mission_active
                    else "관리자 확인을 기록했습니다. 활성 순찰 임무는 없습니다."
                )
                self._publish_incident(record)
                self._update_state(
                    status="executing" if mission_active else "idle",
                    message=mission_message,
                    incident=record,
                )
                return self._decision_response(
                    response,
                    accepted=True,
                    record=record,
                    request_id=request_id,
                    decision=decision,
                    message=mission_message,
                )

            self._publish_dispenser_drop(record, request_id)
            self._publish_incident(record)
            self._update_state(
                status="dispensing",
                message="관리자 승인에 따라 비콘 배출 결과를 기다립니다.",
                incident=record,
            )
            return self._decision_response(
                response,
                accepted=True,
                record=record,
                request_id=request_id,
                decision=decision,
                message="비콘 배출 요청을 전송했습니다.",
            )

    def _publish_dispenser_drop(
        self,
        record: dict[str, Any],
        request_id: str,
    ) -> None:
        detection_id = str(record.get("detection_id") or "") or None
        if bool(self.get_parameter("hazard_approval_enabled").value):
            timeout = max(
                1.0,
                float(
                    self.get_parameter(
                        "dispenser_verification_timeout_sec"
                    ).value
                ),
            )
            self._pending_dispenser_verifications.setdefault(
                request_id,
                {
                    "detection_id": detection_id or "",
                    "deadline": time.monotonic() + timeout,
                    "attempts": 0,
                    "in_flight": False,
                    "future": None,
                    "goal_accepted": False,
                },
            )
        if not self._dispenser_action_client.server_is_ready():
            self.get_logger().error("디스펜서 Action 서버가 준비되지 않았습니다")
            return
        goal = DispenseBeacon.Goal()
        goal.request_id = request_id
        goal.detection_id = detection_id or ""
        goal.authorization = command_authorization(
            self._dispenser_approval_secret,
            request_id=request_id,
            detection_id=detection_id,
        )
        future = self._dispenser_action_client.send_goal_async(
            goal,
            feedback_callback=self._on_dispenser_action_feedback,
        )
        future.add_done_callback(
            lambda completed, rid=request_id: (
                self._on_dispenser_action_goal(rid, completed)
            )
        )

    def _on_dispenser_action_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        self.get_logger().debug(
            f"디스펜서 Action 진행: {feedback.state} {feedback.message}"
        )

    def _on_dispenser_action_goal(self, request_id: str, future: Any) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise RuntimeError("디스펜서 Action 목표가 거부됐습니다")
            with self._state_lock:
                pending = self._pending_dispenser_verifications.get(request_id)
                if pending is not None:
                    pending["goal_accepted"] = True
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, rid=request_id: (
                    self._on_dispenser_action_result(rid, completed)
                )
            )
        except Exception as exc:
            with self._state_lock:
                self._pending_dispenser_verifications.pop(request_id, None)
                self._apply_dispenser_result(
                    {
                        "request_id": request_id,
                        "state": "hardware_error",
                        "result_detail": str(exc),
                        "actuation_started": False,
                    }
                )

    def _on_dispenser_action_result(self, request_id: str, future: Any) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            payload = {
                "request_id": str(result.request_id),
                "detection_id": str(result.detection_id),
                "state": str(result.state),
                "result_detail": str(result.result_detail),
                "dropped_by": str(result.dropped_by),
                "actuation_started": bool(result.actuation_started),
                "home_recovered": bool(result.home_recovered),
            }
            if payload["request_id"] != request_id:
                raise ValueError("디스펜서 Action 결과 request_id가 일치하지 않습니다")
        except Exception as exc:
            payload = {
                "request_id": request_id,
                "state": "hardware_error",
                "result_detail": str(exc),
                "actuation_started": True,
            }
        with self._state_lock:
            self._pending_dispenser_verifications.pop(request_id, None)
            self._apply_dispenser_result(payload)

    def _on_dispenser_result(self, message: String) -> None:
        try:
            result = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(result, dict):
            return
        if bool(self.get_parameter("hazard_approval_enabled").value):
            request_id = str(result.get("request_id") or "")
            with self._state_lock:
                current = self._incident.snapshot()
                if (
                    not request_id
                    or current is None
                    or current.get("state") != "dispensing"
                    or current.get("request_id") != request_id
                ):
                    return
                timeout = max(
                    1.0,
                    float(
                        self.get_parameter(
                            "dispenser_verification_timeout_sec"
                        ).value
                    ),
                )
                self._pending_dispenser_verifications.setdefault(
                    request_id,
                    {
                        "detection_id": str(
                            current.get("detection_id") or ""
                        ),
                        "deadline": time.monotonic() + timeout,
                        "attempts": 0,
                        "in_flight": False,
                        "future": None,
                        "goal_accepted": True,
                    },
                )
            self._poll_pending_dispenser_results()
            return
        with self._state_lock:
            self._apply_dispenser_result(result)

    def _poll_pending_dispenser_results(self) -> None:
        now = time.monotonic()
        timed_out: list[tuple[str, bool]] = []
        with self._state_lock:
            for request_id, pending in list(
                self._pending_dispenser_verifications.items()
            ):
                if now >= float(pending["deadline"]):
                    future = pending.get("future")
                    if future is not None:
                        future.cancel()
                    timed_out.append(
                        (request_id, bool(pending.get("goal_accepted")))
                    )
                    self._pending_dispenser_verifications.pop(
                        request_id, None
                    )
                    continue
                if pending["in_flight"]:
                    continue
                if not self._dispenser_request_status_client.service_is_ready():
                    continue
                request = DispenserRequestStatus.Request()
                request.request_id = request_id
                request.detection_id = str(pending["detection_id"])
                try:
                    future = self._dispenser_request_status_client.call_async(
                        request
                    )
                except Exception as exc:
                    pending["attempts"] += 1
                    self.get_logger().warning(
                        f"디스펜서 원장 조회 시작 실패: {exc}"
                    )
                    continue
                pending["attempts"] += 1
                pending["in_flight"] = True
                pending["future"] = future
                future.add_done_callback(
                    lambda completed, rid=request_id: (
                        self._on_verified_dispenser_result(rid, completed)
                    )
                )
        for request_id, actuation_possible in timed_out:
            self.get_logger().error(
                f"디스펜서 원장 확인 시간 초과: {request_id}"
            )
            with self._state_lock:
                self._apply_dispenser_result(
                    {
                        "request_id": request_id,
                        "state": "hardware_error",
                        "result_detail": "request_ledger_verification_timeout",
                        "actuation_started": actuation_possible,
                    }
                )

    def _on_verified_dispenser_result(
        self,
        request_id: str,
        future: Any,
    ) -> None:
        try:
            response = future.result()
            if not response.found:
                raise ValueError("요청 원장에 배출 결과가 없습니다")
            if not response.fingerprint_matches:
                with self._state_lock:
                    self._pending_dispenser_verifications.pop(
                        request_id, None
                    )
                    self._apply_dispenser_result(
                        {
                            "request_id": request_id,
                            "state": "idempotency_conflict",
                            "result_detail": "ledger_detection_id_mismatch",
                            "actuation_started": True,
                        }
                    )
                return
            record = json.loads(response.record_json)
            if not isinstance(record, dict):
                raise ValueError("요청 원장 결과가 객체가 아닙니다")
            if str(record.get("request_id") or "") != request_id:
                raise ValueError("요청 원장 request_id가 일치하지 않습니다")
        except Exception as exc:
            with self._state_lock:
                pending = self._pending_dispenser_verifications.get(request_id)
                if pending is not None:
                    pending["in_flight"] = False
                    pending["future"] = None
            self.get_logger().warning(f"디스펜서 원장 결과 검증 재시도: {exc}")
            return
        with self._state_lock:
            self._apply_dispenser_result(record)
            pending = self._pending_dispenser_verifications.get(request_id)
            if pending is None:
                return
            if str(record.get("state") or "") in PROGRESS_DISPENSER_STATES:
                pending["in_flight"] = False
                pending["future"] = None
            else:
                self._pending_dispenser_verifications.pop(request_id, None)

    def _apply_dispenser_result(self, result: dict[str, Any]) -> None:
        current = self._incident.snapshot()
        if current is None or current.get("state") != "dispensing":
            return
        request_id = str(result.get("request_id") or "")
        if request_id != str(current.get("request_id") or ""):
            return
        result_state = str(result.get("state") or "hardware_error")
        detail = str(result.get("result_detail") or "")
        try:
            if result_state in PROGRESS_DISPENSER_STATES:
                actuation_value = result.get("actuation_started")
                actuation_started = (
                    actuation_value if isinstance(actuation_value, bool) else None
                )
                updated = self._incident.mark_dispense_progress(
                    dispenser_request_id=request_id,
                    state=result_state,
                    actuation_started=actuation_started,
                )
                self._publish_incident(updated)
                self._update_state(
                    status="dispensing",
                    message=f"비콘 배출 진행 중: {result_state}",
                    incident=updated,
                )
                return
            if result_state == "succeeded":
                beacon_pose = self._current_beacon_pose(current)
                updated = self._incident.mark_dispense_succeeded(
                    dispenser_request_id=request_id,
                    result_detail=detail,
                    beacon_pose=beacon_pose,
                )
                if updated["state"] == "resuming":
                    updated = self._incident.resolve_resume()
                    mission_status = (
                        "executing" if self._mission_active else "idle"
                    )
                    mission_message = (
                        "비콘 배출을 확인하고 순찰을 재개합니다."
                        if self._mission_active
                        else "비콘 배출을 확인했습니다. 활성 순찰 임무는 없습니다."
                    )
                else:
                    mission_status = "monitoring"
                    mission_message = (
                        "비콘 설치 지점 감시 중입니다. 관리자 확인 전에는 재개하지 않습니다."
                    )
                    self._begin_monitoring(updated)
            else:
                known_terminal = result_state in TERMINAL_DISPENSER_STATES
                actuation_value = result.get("actuation_started")
                actuation_started = (
                    actuation_value
                    if known_terminal and isinstance(actuation_value, bool)
                    else True
                )
                updated = self._incident.mark_dispense_failed(
                    dispenser_request_id=request_id,
                    result=(result_state if known_terminal else "hardware_error"),
                    result_detail=detail,
                    actuation_started=actuation_started,
                    beacon_pose=(
                        self._current_beacon_pose(current)
                        if actuation_started
                        else None
                    ),
                )
                mission_status = str(updated["state"])
                mission_message = (
                    "비콘 배출 결과를 확인해야 하므로 순찰을 정지 상태로 유지합니다."
                    if updated["state"] != "approval_required"
                    else "비콘이 동작하기 전에 실패했습니다. 관리자 결정을 다시 선택할 수 있습니다."
                )
        except (ValueError, IncidentConflictError) as exc:
            self.get_logger().error(f"디스펜서 결과 상태 전이 실패: {exc}")
            return
        updated["message"] = mission_message
        self._publish_incident(updated)
        self._update_state(
            status=mission_status,
            message=mission_message,
            incident=updated,
        )

    def _current_beacon_pose(self, incident: dict[str, Any]) -> dict[str, Any]:
        frame_id = str(incident.get("frame_id") or "map")
        pose = self._nav.current_pose(frame_id)
        if pose is None:
            self.get_logger().warning(
                "배출 시점 로봇 위치를 확인하지 못해 비콘 위치를 기록하지 않습니다"
            )
            return {
                "beacon_pose_available": False,
                "beacon_frame_id": frame_id,
            }
        x, y, yaw = pose
        offset = float(self.get_parameter("dispenser_rear_offset_m").value)
        return {
            "beacon_pose_available": True,
            "beacon_frame_id": frame_id,
            "beacon_x": x + offset * math.cos(yaw),
            "beacon_y": y + offset * math.sin(yaw),
            "beacon_z": 0.0,
            "beacon_yaw": yaw,
        }

    def _begin_monitoring(self, incident: dict[str, Any]) -> None:
        self._set_thermal_focus(str(incident.get("equipment_id") or "") or None)
        with self._monitoring_lock:
            self._monitoring_phase = "idle"
            self._monitoring_due_at = time.monotonic()
            self._monitoring_pending_correlation = None
            self._monitoring_pending_deadline = 0.0
            self._monitoring_failures = 0

    def _reset_monitoring_state(self, *, clear_focus: bool = False) -> None:
        with self._monitoring_lock:
            pending = self._monitoring_pending_correlation
            self._monitoring_phase = "idle"
            self._monitoring_due_at = 0.0
            self._monitoring_pending_correlation = None
            self._monitoring_pending_deadline = 0.0
            self._monitoring_failures = 0
        if pending:
            with self._thermal_trend_condition:
                self._expected_thermal_correlations.discard(pending)
                self._thermal_completed_correlations.discard(pending)
        if clear_focus:
            self._set_thermal_focus(None)

    def _monitoring_tick(self) -> None:
        if not self._monitoring_lock.acquire(blocking=False):
            return
        try:
            incident = self._incident.snapshot()
            if incident is None or incident.get("state") != "monitoring":
                return
            now = time.monotonic()
            if self._monitoring_pending_correlation:
                if now < self._monitoring_pending_deadline:
                    return
                correlation_id = self._monitoring_pending_correlation
                self._monitoring_pending_correlation = None
                self._monitoring_pending_deadline = 0.0
                self._monitoring_due_at = now + max(
                    0.5,
                    float(self.get_parameter("monitoring_interval_sec").value),
                )
                with self._thermal_trend_condition:
                    self._expected_thermal_correlations.discard(correlation_id)
                    self._thermal_completed_correlations.discard(correlation_id)
                self.get_logger().warning(
                    "현장 감시 열화상 판정 시간이 초과되어 다음 수집을 예약합니다"
                )
                self._handle_monitoring_failure("thermal_result_timeout")
                return
            if now < self._monitoring_due_at:
                return
            if self._monitoring_phase == "idle":
                if not self._start_thermal_visit("incident monitoring"):
                    self._monitoring_due_at = now + max(
                        0.5,
                        float(
                            self.get_parameter("monitoring_interval_sec").value
                        ),
                    )
                    self._handle_monitoring_failure("thermal_start_unavailable")
                    return
                self._monitoring_phase = "collecting"
                self._monitoring_due_at = now + max(
                    0.1,
                    float(self.get_parameter("monitoring_dwell_sec").value),
                )
                return
            correlation_id = (
                f"monitor:{incident['incident_id']}:{uuid.uuid4().hex}"
            )
            with self._thermal_trend_condition:
                self._expected_thermal_correlations.add(correlation_id)
            visit_index = self._record_thermal_visit(
                "incident monitoring", correlation_id
            )
            self._monitoring_phase = "idle"
            if visit_index is None:
                with self._thermal_trend_condition:
                    self._expected_thermal_correlations.discard(correlation_id)
                self._monitoring_due_at = now + max(
                    0.5,
                    float(self.get_parameter("monitoring_interval_sec").value),
                )
                self._handle_monitoring_failure("thermal_record_unavailable")
                return
            self._monitoring_failures = 0
            self._monitoring_pending_correlation = correlation_id
            self._monitoring_pending_deadline = now + max(
                0.5,
                float(
                    self.get_parameter("monitoring_result_timeout_sec").value
                ),
            )
        finally:
            self._monitoring_lock.release()

    def _handle_monitoring_failure(self, reason: str) -> None:
        self._monitoring_failures += 1
        maximum = max(
            1,
            int(self.get_parameter("monitoring_max_failures").value),
        )
        if self._monitoring_failures < maximum:
            return
        try:
            updated = self._incident.mark_monitoring_release_required(
                message=(
                    "열화상 현장 감시 통신을 확인할 수 없습니다. "
                    "관리자 판단 후 순찰 재개 여부를 결정하세요."
                ),
                normalized=False,
            )
        except IncidentConflictError:
            return
        updated["monitoring_failure"] = reason
        self._publish_incident(updated)
        self._update_state(
            status="admin_release_required",
            message=updated["message"],
            incident=updated,
        )
        self._set_thermal_focus(None)

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
        with self._state_lock:
            self._cancel_requested.set()
            incident = self._incident.snapshot()
            message = (
                "순찰 중단을 요청했습니다. 활성 위험 이벤트는 관리자 확인 전까지 유지됩니다."
                if incident is not None and self._incident.is_paused()
                else "순찰 중단을 요청했습니다."
            )
            self._update_state(
                status="canceling",
                accepted=False,
                message=message,
                incident=incident,
            )
        self._nav.cancel_active()

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
        with self._thermal_trend_condition:
            self._expected_thermal_correlations.clear()
            self._thermal_completed_correlations.clear()
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

                thermal_required = bool(
                    self.get_parameter("hazard_approval_enabled").value
                )
                thermal_started = self._start_thermal_visit(
                    f"cycle {current_cycle}"
                )
                if thermal_required and not thermal_started:
                    raise MissionFailure(
                        "열화상 방문 수집을 시작하지 못해 순찰을 중단합니다"
                    )

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

                correlation_id = (
                    f"{request.mission_id}:cycle-{current_cycle}:"
                    f"{uuid.uuid4().hex}"
                )
                with self._thermal_trend_condition:
                    self._expected_thermal_correlations.add(correlation_id)
                recorded_visit_index = self._record_thermal_visit(
                    f"cycle {current_cycle}",
                    correlation_id,
                )
                if recorded_visit_index is None and thermal_required:
                    with self._thermal_trend_condition:
                        self._expected_thermal_correlations.discard(
                            correlation_id
                        )
                    raise MissionFailure(
                        "열화상 방문 기록을 저장하지 못해 순찰을 중단합니다"
                    )
                if recorded_visit_index is not None and thermal_required:
                    try:
                        self._wait_for_thermal_evaluation(
                            goal_handle,
                            correlation_id,
                        )
                        self._wait_for_safety_clear(goal_handle)
                    finally:
                        with self._thermal_trend_condition:
                            self._expected_thermal_correlations.discard(
                                correlation_id
                            )
                            self._thermal_completed_correlations.discard(
                                correlation_id
                            )
                else:
                    with self._thermal_trend_condition:
                        self._expected_thermal_correlations.discard(
                            correlation_id
                        )

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

            self._seal_successful_mission(goal_handle)
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
            self._close_mission_callback_gate()
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
            self._close_mission_callback_gate()
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
            self._close_mission_callback_gate()
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
            self._close_mission_callback_gate()
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

    def _call_thermal_service(
        self,
        client: Any,
        label: str,
        request: Any | None = None,
    ) -> Any | None:
        """Complete one thermal visit transition before the next can start."""

        if self._thermal_sequence_faulted:
            return None
        if not client.service_is_ready():
            self.get_logger().info(f"{label}: thermal service is not active")
            return None

        future = client.call_async(request or Trigger.Request())
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
            return None
        try:
            response = future.result()
        except Exception as exc:
            self._thermal_sequence_faulted = True
            self.get_logger().warning(f"{label}: thermal service failed: {exc}")
            return None
        if not response.success:
            self.get_logger().warning(f"{label}: {response.message}")
            return None
        self.get_logger().info(f"{label}: {response.message}")
        return response

    def _start_thermal_visit(self, cycle_name: str) -> bool:
        """Reset the thermal accumulator once before a patrol cycle."""

        return self._call_thermal_service(
            self._thermal_start_client,
            f"{cycle_name}: thermal visit start",
        ) is not None

    def _record_thermal_visit(
        self,
        cycle_name: str,
        correlation_id: str,
    ) -> int | None:
        """Persist a completed visit before another patrol cycle can start."""

        request = RecordThermalVisit.Request()
        request.correlation_id = correlation_id
        response = self._call_thermal_service(
            self._thermal_record_client,
            f"{cycle_name}: thermal visit record",
            request,
        )
        if response is None:
            return None
        visit_index = int(response.visit_index)
        if visit_index < 1:
            self.get_logger().error(
                f"{cycle_name}: thermal record response has invalid visit_index"
            )
            return None
        return visit_index

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
        announced: tuple[str, str] | None = None
        while self._operational_is_paused():
            incident = self._incident.snapshot()
            if incident is not None and self._incident.is_paused():
                self._raise_if_user_canceled(goal_handle)
                pause = (
                    str(incident["state"]),
                    str(
                        incident.get("message")
                        or "위험 이벤트 관리자 승인을 기다립니다."
                    ),
                )
            else:
                self._raise_if_canceled(goal_handle)
                _state, reason = self._safety.snapshot()
                pause = (
                    "safety_paused",
                    (
                        "사람 안전 구역이 확보될 때까지 대기합니다. "
                        f"{reason}"
                    ).strip(),
                )
            if pause != announced:
                if pause[0] == "safety_paused":
                    self._update_state(
                        status=pause[0],
                        message=pause[1],
                    )
                else:
                    self._update_state(
                        status=pause[0],
                        message=pause[1],
                        incident=incident,
                    )
                announced = pause
            time.sleep(0.05)
        if announced is not None:
            self._update_state(
                status="executing",
                message="안전 구역이 확보되어 현재 목적지를 다시 계획합니다.",
            )

    def _wait_for_thermal_evaluation(
        self,
        goal_handle: Any,
        expected_correlation_id: str,
    ) -> None:
        timeout = float(
            self.get_parameter("hazard_evaluation_timeout_sec").value
        )
        deadline = time.monotonic() + max(0.0, timeout)
        with self._thermal_trend_condition:
            while (
                expected_correlation_id
                not in self._thermal_completed_correlations
            ):
                self._raise_if_canceled(goal_handle)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.get_logger().warning(
                        "열화상 위험 판정 결과 대기 시간이 초과됐습니다"
                    )
                    raise MissionFailure(
                        "열화상 위험 판정 결과를 확인하지 못해 순찰을 중단합니다"
                    )
                self._thermal_trend_condition.wait(
                    timeout=min(0.1, remaining)
                )

    def _seal_successful_mission(self, goal_handle: Any) -> None:
        """Atomically stop accepting incident callbacks or wait for approval."""

        while True:
            self._wait_for_safety_clear(goal_handle)
            with self._state_lock:
                if self._incident.is_paused():
                    continue
                self._mission_active = False
                return

    def _close_mission_callback_gate(self) -> None:
        with self._state_lock:
            self._mission_active = False

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
        self._raise_if_user_canceled(goal_handle)
        schedule = self._active_schedule
        if schedule is not None and schedule.deadline_reached():
            raise MissionScheduleEnded()

    def _raise_if_user_canceled(self, goal_handle: Any) -> None:
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
        self._reset_monitoring_state(clear_focus=True)
        self._dispenser_action_client.destroy()
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
