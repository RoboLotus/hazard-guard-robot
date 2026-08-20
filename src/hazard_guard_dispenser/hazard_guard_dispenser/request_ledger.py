"""Durable, fail-closed idempotency records for physical dispenser requests."""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TERMINAL_STATES = frozenset(
    {
        "succeeded",
        "jam_suspected",
        "hardware_error",
        "canceled",
        "rejected_busy",
        "recovery_required",
        "command_completed_unverified",
    }
)
IN_PROGRESS_STATES = frozenset({"accepted", "dispensing", "waiting", "homing"})


class RequestLedgerError(RuntimeError):
    """Raised when durable idempotency state cannot be safely maintained."""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


class RequestLedger:
    """File-backed request ledger with atomic writes.

    A physical command is never re-issued for an existing request id or
    detection id. Interrupted records are intentionally recovered as
    ``recovery_required``: the process cannot know whether a servo movement
    happened immediately before it exited.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self._lock = threading.RLock()
        self._records: dict[str, dict[str, Any]] = {}
        self._load()
        self.recover_interrupted()

    def _load(self) -> None:
        with self._lock:
            if not self.path.exists():
                self._records = {}
                return
            try:
                data = json.loads(self.path.read_text(encoding="utf-8"))
                records = data.get("records") if isinstance(data, dict) else None
                if not isinstance(records, dict):
                    raise ValueError("records must be an object")
                self._records = {
                    str(key): dict(value)
                    for key, value in records.items()
                    if isinstance(value, dict)
                }
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                raise RequestLedgerError(
                    f"디스펜서 요청 원장을 읽을 수 없습니다: {exc}"
                ) from exc

    def _write(self) -> None:
        try:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            payload = {"schema_version": 1, "records": self._records}
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
            if os.name != "nt":
                self.path.chmod(0o600)
        except OSError as exc:
            raise RequestLedgerError(
                f"디스펜서 요청 원장을 저장할 수 없습니다: {exc}"
            ) from exc

    @staticmethod
    def _copy(record: dict[str, Any]) -> dict[str, Any]:
        return json.loads(json.dumps(record, ensure_ascii=False))

    def _find_detection(self, detection_id: str) -> dict[str, Any] | None:
        for record in self._records.values():
            if record.get("detection_id") == detection_id:
                return record
        return None

    def claim(
        self, *, request_id: str, detection_id: str | None, command: str = "drop"
    ) -> tuple[dict[str, Any], bool]:
        """Persist a request before an actuator is allowed to move."""

        with self._lock:
            current = self._records.get(request_id)
            if current is not None:
                return self._copy(current), False
            if detection_id:
                existing_detection = self._find_detection(detection_id)
                if existing_detection is not None:
                    return self._copy(existing_detection), False
            record = {
                "request_id": request_id,
                "detection_id": detection_id,
                "command": command,
                "state": "accepted",
                "created_at": _now(),
                "updated_at": _now(),
            }
            self._records[request_id] = record
            self._write()
            return self._copy(record), True

    def transition(
        self, request_id: str, state: str, **fields: Any
    ) -> dict[str, Any]:
        with self._lock:
            record = self._records.get(request_id)
            if record is None:
                raise RequestLedgerError(f"알 수 없는 request_id: {request_id}")
            record["state"] = state
            record["updated_at"] = _now()
            record.update(fields)
            self._write()
            return self._copy(record)

    def recover_interrupted(self) -> list[dict[str, Any]]:
        """Do not replay commands whose physical outcome was interrupted."""

        with self._lock:
            recovered = []
            for record in self._records.values():
                if record.get("state") not in IN_PROGRESS_STATES:
                    continue
                record.update(
                    {
                        "state": "recovery_required",
                        "result_detail": "process_restarted_before_terminal_result",
                        "updated_at": _now(),
                    }
                )
                recovered.append(self._copy(record))
            if recovered:
                self._write()
            return recovered

    def get(self, request_id: str) -> dict[str, Any] | None:
        with self._lock:
            record = self._records.get(request_id)
            return self._copy(record) if record is not None else None
