"""Durable, fail-closed idempotency records for physical dispenser requests."""

from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TERMINAL_STATES = frozenset({
    "succeeded", "jam_suspected", "hardware_error", "canceled",
    "rejected_busy", "recovery_required", "command_completed_unverified",
    "hardware_unavailable", "rejected_no_confirmation", "safety_interlock",
    "idempotency_conflict",
})
IN_PROGRESS_STATES = frozenset({"accepted", "arming", "dispensing", "waiting", "homing"})
_PROGRESS_ORDER = {
    "accepted": 0,
    "arming": 1,
    "dispensing": 2,
    "waiting": 3,
    "homing": 4,
}


class RequestLedgerError(RuntimeError):
    """Raised when durable idempotency state cannot be safely maintained."""


class IdempotencyConflictError(RequestLedgerError):
    """Raised when an idempotency key is reused for a different operation."""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def request_fingerprint(command: str, detection_id: str | None) -> str:
    canonical = json.dumps(
        {"command": command, "detection_id": detection_id},
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


class RequestLedger:
    """SQLite-backed request ledger with one transaction per state change."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self._lock = threading.RLock()
        try:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            self._initialize()
            self.recover_interrupted()
        except (OSError, sqlite3.Error, ValueError, TypeError) as exc:
            raise RequestLedgerError(f"디스펜서 요청 원장을 열 수 없습니다: {exc}") from exc

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=5.0, isolation_level=None)
        try:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("PRAGMA busy_timeout=5000")
            return connection
        except Exception:
            connection.close()
            raise

    def _initialize(self) -> None:
        with closing(self._connect()) as connection, connection:
            connection.execute(
                """CREATE TABLE IF NOT EXISTS dispenser_requests (
                    request_id TEXT PRIMARY KEY,
                    detection_id TEXT UNIQUE,
                    state TEXT NOT NULL,
                    record_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )"""
            )

    @staticmethod
    def _validate(record: dict[str, Any]) -> dict[str, Any]:
        required = ("request_id", "command", "state", "created_at", "updated_at")
        if not all(isinstance(record.get(key), str) and record[key] for key in required):
            raise RequestLedgerError("원장 레코드 스키마가 손상되었습니다")
        if record["state"] not in TERMINAL_STATES | IN_PROGRESS_STATES:
            raise RequestLedgerError("원장 레코드 상태가 유효하지 않습니다")
        if record.get("detection_id") is not None and not isinstance(record["detection_id"], str):
            raise RequestLedgerError("원장 detection_id가 손상되었습니다")
        return record

    def _decode(self, encoded: str) -> dict[str, Any]:
        try:
            decoded = json.loads(encoded)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RequestLedgerError("원장 레코드 JSON이 손상되었습니다") from exc
        if not isinstance(decoded, dict):
            raise RequestLedgerError("원장 레코드가 객체가 아닙니다")
        return self._validate(decoded)

    def _row_record(self, row) -> dict[str, Any] | None:
        return self._decode(row[0]) if row is not None else None

    @staticmethod
    def _encode(record: dict[str, Any]) -> str:
        return json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))

    @staticmethod
    def _can_transition(current: str, next_state: str) -> bool:
        if current in TERMINAL_STATES:
            return False
        if next_state in TERMINAL_STATES:
            return True
        return _PROGRESS_ORDER.get(next_state, -1) >= _PROGRESS_ORDER.get(current, -1)

    def claim(self, *, request_id: str, detection_id: str | None, command: str = "drop") -> tuple[dict[str, Any], bool]:
        with self._lock, closing(self._connect()) as connection, connection:
            connection.execute("BEGIN IMMEDIATE")
            request_row = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE request_id = ?", (request_id,)
            ).fetchone()
            detection_row = None
            if detection_id:
                detection_row = connection.execute(
                    "SELECT record_json FROM dispenser_requests WHERE detection_id = ?", (detection_id,)
                ).fetchone()
            by_request = self._row_record(request_row)
            by_detection = self._row_record(detection_row)
            if (
                by_request is not None
                and by_detection is not None
                and by_request["request_id"] != by_detection["request_id"]
            ):
                connection.rollback()
                raise IdempotencyConflictError(
                    "request_id와 detection_id가 서로 다른 기존 요청을 가리킵니다"
                )
            existing = by_request or by_detection
            if existing is not None:
                expected = request_fingerprint(command, detection_id)
                actual = existing.get("request_fingerprint") or request_fingerprint(
                    existing["command"], existing.get("detection_id")
                )
                if by_request is not None and actual != expected:
                    connection.rollback()
                    raise IdempotencyConflictError(
                        "request_id가 다른 배출 요청 내용으로 재사용되었습니다"
                    )
                if existing["command"] != command:
                    connection.rollback()
                    raise IdempotencyConflictError(
                        "detection_id가 다른 명령으로 재사용되었습니다"
                    )
                if existing["state"] in {
                    "rejected_busy",
                    "hardware_unavailable",
                    "rejected_no_confirmation",
                    "safety_interlock",
                }:
                    existing.update(
                        state="accepted",
                        updated_at=_now(),
                        result_detail="safe_retry_before_actuation",
                    )
                    connection.execute(
                        "UPDATE dispenser_requests SET state=?, record_json=?, updated_at=? WHERE request_id=?",
                        (
                            existing["state"],
                            self._encode(existing),
                            existing["updated_at"],
                            existing["request_id"],
                        ),
                    )
                    connection.commit()
                    return existing, True
                connection.commit()
                return existing, False
            timestamp = _now()
            record = {
                "request_id": request_id,
                "detection_id": detection_id,
                "command": command,
                "request_fingerprint": request_fingerprint(command, detection_id),
                "state": "accepted",
                "created_at": timestamp,
                "updated_at": timestamp,
            }
            connection.execute(
                "INSERT INTO dispenser_requests VALUES (?, ?, ?, ?, ?, ?)",
                (request_id, detection_id, "accepted", self._encode(record), timestamp, timestamp),
            )
            connection.commit()
            return record, True

    def transition(self, request_id: str, state: str, **fields: Any) -> dict[str, Any]:
        with self._lock, closing(self._connect()) as connection, connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE request_id = ?", (request_id,)
            ).fetchone()
            record = self._row_record(row)
            if record is None:
                raise RequestLedgerError(f"알 수 없는 request_id: {request_id}")
            if not self._can_transition(record["state"], state):
                connection.commit()
                return record
            record.update(fields)
            record["state"] = state
            record["updated_at"] = _now()
            connection.execute(
                "UPDATE dispenser_requests SET state=?, record_json=?, updated_at=? WHERE request_id=?",
                (state, self._encode(record), record["updated_at"], request_id),
            )
            connection.commit()
            return record

    def recover_interrupted(self) -> list[dict[str, Any]]:
        with self._lock, closing(self._connect()) as connection, connection:
            connection.execute("BEGIN IMMEDIATE")
            rows = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE state IN ('accepted','arming','dispensing','waiting','homing')"
            ).fetchall()
            recovered = []
            for row in rows:
                record = self._row_record(row)
                record.update({
                    "state": "recovery_required",
                    "result_detail": "process_restarted_before_terminal_result",
                    "updated_at": _now(),
                })
                connection.execute(
                    "UPDATE dispenser_requests SET state=?, record_json=?, updated_at=? WHERE request_id=?",
                    (record["state"], self._encode(record), record["updated_at"], record["request_id"]),
                )
                recovered.append(record)
            connection.commit()
            return recovered

    def get(self, request_id: str) -> dict[str, Any] | None:
        with self._lock, closing(self._connect()) as connection, connection:
            row = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE request_id = ?", (request_id,)
            ).fetchone()
            return self._row_record(row)
