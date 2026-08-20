"""Durable, fail-closed idempotency records for physical dispenser requests."""

from __future__ import annotations

import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TERMINAL_STATES = frozenset({
    "succeeded", "jam_suspected", "hardware_error", "canceled",
    "rejected_busy", "recovery_required", "command_completed_unverified",
})
IN_PROGRESS_STATES = frozenset({"accepted", "dispensing", "waiting", "homing"})
_PROGRESS_ORDER = {"accepted": 0, "dispensing": 1, "waiting": 2, "homing": 3}


class RequestLedgerError(RuntimeError):
    """Raised when durable idempotency state cannot be safely maintained."""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


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
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=FULL")
        connection.execute("PRAGMA busy_timeout=5000")
        return connection

    def _initialize(self) -> None:
        with self._connect() as connection:
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
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE request_id = ?", (request_id,)
            ).fetchone()
            if row is None and detection_id:
                row = connection.execute(
                    "SELECT record_json FROM dispenser_requests WHERE detection_id = ?", (detection_id,)
                ).fetchone()
            existing = self._row_record(row)
            if existing is not None:
                connection.commit()
                return existing, False
            timestamp = _now()
            record = {
                "request_id": request_id,
                "detection_id": detection_id,
                "command": command,
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
        with self._lock, self._connect() as connection:
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
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            rows = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE state IN ('accepted','dispensing','waiting','homing')"
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
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT record_json FROM dispenser_requests WHERE request_id = ?", (request_id,)
            ).fetchone()
            return self._row_record(row)
