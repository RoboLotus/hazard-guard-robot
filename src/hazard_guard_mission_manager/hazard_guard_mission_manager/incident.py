from __future__ import annotations

import copy
import re
import threading
from typing import Any


DECISION_RESUME = "resume"
DECISION_DROP_THEN_RESUME = "drop_then_resume"
DECISION_DROP_THEN_MONITOR = "drop_then_monitor"
DECISION_COMPLETE_MONITORING = "complete_monitoring"

DECISIONS = {
    DECISION_RESUME,
    DECISION_DROP_THEN_RESUME,
    DECISION_DROP_THEN_MONITOR,
    DECISION_COMPLETE_MONITORING,
}

TERMINAL_DISPENSER_STATES = {
    "succeeded",
    "jam_suspected",
    "hardware_error",
    "canceled",
    "safety_interlock",
    "rejected_no_confirmation",
    "idempotency_conflict",
}
REQUEST_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,96}$")


class IncidentConflictError(RuntimeError):
    pass


class IncidentApprovalLatch:
    """Thread-safe approval hold independent from ROS-generated classes."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._incident: dict[str, Any] | None = None
        self._decision_requests: dict[str, dict[str, Any]] = {}

    def snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            return copy.deepcopy(self._incident)

    def is_paused(self) -> bool:
        with self._lock:
            return self._incident is not None and self._incident["state"] not in {
                "resolved",
                "canceled",
            }

    def open(self, incident: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        incident_id = str(incident.get("incident_id", "")).strip()
        if not incident_id:
            raise ValueError("incident_id가 필요합니다")
        with self._lock:
            if self._incident is not None:
                if self._incident["incident_id"] == incident_id:
                    return copy.deepcopy(self._incident), False
                if self.is_paused():
                    raise IncidentConflictError("다른 위험 이벤트가 승인 대기 중입니다")
            record = copy.deepcopy(incident)
            record.update(
                incident_id=incident_id,
                state="approval_required",
                decision=None,
                request_id=None,
                operator_id=None,
                decision_history=[],
            )
            self._incident = record
            return copy.deepcopy(record), True

    def decide(
        self,
        *,
        incident_id: str,
        request_id: str,
        decision: str,
        operator_id: str,
    ) -> tuple[dict[str, Any], bool]:
        incident_id = str(incident_id).strip()
        request_id = str(request_id).strip()
        operator_id = str(operator_id).strip()
        if decision not in DECISIONS:
            raise ValueError(f"지원하지 않는 관리자 결정입니다: {decision}")
        if not incident_id:
            raise ValueError("incident_id가 필요합니다")
        if not request_id:
            raise ValueError("request_id가 필요합니다")
        if not REQUEST_ID_PATTERN.fullmatch(request_id):
            raise ValueError("request_id 형식이 올바르지 않습니다")
        if not operator_id:
            raise ValueError("operator_id가 필요합니다")
        with self._lock:
            previous = self._decision_requests.get(request_id)
            fingerprint = {
                "incident_id": incident_id,
                "decision": decision,
                "operator_id": operator_id,
            }
            if previous is not None:
                if previous["fingerprint"] != fingerprint:
                    raise IncidentConflictError("request_id가 다른 결정에 재사용되었습니다")
                return copy.deepcopy(previous["response"]), False
            if self._incident is None or self._incident["incident_id"] != incident_id:
                raise IncidentConflictError("활성 위험 이벤트가 일치하지 않습니다")
            state = self._incident["state"]
            if decision == DECISION_COMPLETE_MONITORING:
                if state not in {"monitoring", "admin_release_required"}:
                    raise IncidentConflictError("감시 중인 이벤트가 아닙니다")
                next_state = "resuming"
            else:
                if state != "approval_required":
                    raise IncidentConflictError("관리자 결정을 받을 수 없는 상태입니다")
                next_state = (
                    "resuming" if decision == DECISION_RESUME else "dispensing"
                )
            decision_entry = {
                "request_id": request_id,
                "decision": decision,
                "operator_id": operator_id,
            }
            self._incident["decision_history"].append(decision_entry)
            if decision == DECISION_COMPLETE_MONITORING:
                self._incident.update(
                    release_request_id=request_id,
                    release_operator_id=operator_id,
                    state=next_state,
                )
            else:
                self._incident.update(
                    decision=decision,
                    request_id=request_id,
                    operator_id=operator_id,
                    state=next_state,
                )
            response = copy.deepcopy(self._incident)
            self._decision_requests[request_id] = {
                "fingerprint": fingerprint,
                "response": response,
            }
            return response, True

    def mark_dispense_succeeded(
        self,
        *,
        dispenser_request_id: str,
        result_detail: str = "",
    ) -> dict[str, Any]:
        dispenser_request_id = str(dispenser_request_id).strip()
        if not dispenser_request_id:
            raise ValueError("dispenser_request_id가 필요합니다")
        with self._lock:
            if self._incident is None:
                raise IncidentConflictError("활성 위험 이벤트가 없습니다")
            if self._incident["state"] != "dispensing":
                raise IncidentConflictError("배출 진행 중인 이벤트가 아닙니다")
            decision = self._incident.get("decision")
            if decision == DECISION_DROP_THEN_RESUME:
                state = "resuming"
            elif decision == DECISION_DROP_THEN_MONITOR:
                state = "monitoring"
            else:
                raise IncidentConflictError("배출을 승인한 관리자 결정이 없습니다")
            self._incident.update(
                state=state,
                dispenser_request_id=dispenser_request_id,
                dispenser_result="succeeded",
                result_detail=str(result_detail),
            )
            return copy.deepcopy(self._incident)

    def mark_dispense_failed(
        self,
        *,
        dispenser_request_id: str,
        result: str,
        result_detail: str = "",
        actuation_started: bool,
    ) -> dict[str, Any]:
        dispenser_request_id = str(dispenser_request_id).strip()
        if not dispenser_request_id:
            raise ValueError("dispenser_request_id가 필요합니다")
        with self._lock:
            if self._incident is None or self._incident["state"] != "dispensing":
                raise IncidentConflictError("배출 진행 중인 이벤트가 아닙니다")
            if actuation_started:
                state = (
                    "field_check_required"
                    if result == "jam_suspected"
                    else "hardware_error"
                )
            else:
                state = "approval_required"
            self._incident.update(
                state=state,
                dispenser_request_id=dispenser_request_id,
                dispenser_result=str(result),
                result_detail=str(result_detail),
            )
            if not actuation_started:
                self._incident.update(
                    decision=None,
                    request_id=None,
                    operator_id=None,
                )
            return copy.deepcopy(self._incident)

    def mark_monitoring_normalized(self, message: str = "") -> dict[str, Any]:
        with self._lock:
            if self._incident is None or self._incident["state"] != "monitoring":
                raise IncidentConflictError("감시 중인 이벤트가 아닙니다")
            self._incident["state"] = "admin_release_required"
            if message:
                self._incident["message"] = str(message)
            return copy.deepcopy(self._incident)

    def resolve_resume(self) -> dict[str, Any]:
        with self._lock:
            if self._incident is None or self._incident["state"] != "resuming":
                raise IncidentConflictError("승인된 재개 상태가 아닙니다")
            self._incident["state"] = "resolved"
            return copy.deepcopy(self._incident)

    def cancel(self, reason: str = "") -> dict[str, Any] | None:
        with self._lock:
            if self._incident is None:
                return None
            self._incident.update(state="canceled", message=str(reason))
            return copy.deepcopy(self._incident)
