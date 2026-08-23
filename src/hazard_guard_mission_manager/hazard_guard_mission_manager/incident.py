from __future__ import annotations

import copy
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
        if decision not in DECISIONS:
            raise ValueError(f"지원하지 않는 관리자 결정입니다: {decision}")
        if not request_id:
            raise ValueError("request_id가 필요합니다")
        with self._lock:
            previous = self._decision_requests.get(request_id)
            fingerprint = {
                "incident_id": incident_id,
                "decision": decision,
                "operator_id": operator_id,
            }
            if previous is not None:
                if previous != fingerprint:
                    raise IncidentConflictError("request_id가 다른 결정에 재사용되었습니다")
                return copy.deepcopy(self._incident), False
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
            self._decision_requests[request_id] = fingerprint
            self._incident.update(
                decision=decision,
                request_id=request_id,
                operator_id=operator_id,
                state=next_state,
            )
            return copy.deepcopy(self._incident), True

    def transition(self, state: str, **values: Any) -> dict[str, Any]:
        with self._lock:
            if self._incident is None:
                raise IncidentConflictError("활성 위험 이벤트가 없습니다")
            self._incident.update(state=state, **values)
            return copy.deepcopy(self._incident)
