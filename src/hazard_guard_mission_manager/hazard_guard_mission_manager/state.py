from __future__ import annotations

import copy
import json
import threading
from collections.abc import Callable
from typing import Any


def initial_mission_state() -> dict[str, Any]:
    return {
        "mission_id": None,
        "name": None,
        "status": "idle",
        "accepted": False,
        "mock": False,
        "frame_id": "map",
        "current_index": None,
        "total_waypoints": 0,
        "completed_waypoints": 0,
        "repeat_mode": "once",
        "repeat_count": 1,
        "repeat_interval_sec": 0.0,
        "current_cycle": 0,
        "total_cycles": 1,
        "completed_cycles": 0,
        "start_at_unix_ms": 0,
        "end_at_unix_ms": 0,
        "next_run_at_unix_ms": 0,
        "total_distance_m": None,
        "message": "실행 중인 순찰 임무가 없습니다.",
        "waypoints": [],
    }


class MissionStateStore:
    """Thread-safe mission state with a single change notification boundary."""

    def __init__(
        self,
        on_change: Callable[[str], None],
        initial_state: dict[str, Any] | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._on_change = on_change
        self._state = copy.deepcopy(initial_state or initial_mission_state())

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._state)

    def replace(self, state: dict[str, Any]) -> None:
        with self._lock:
            self._state = copy.deepcopy(state)
            payload = self._payload_locked()
        self._on_change(payload)

    def update(self, **values: Any) -> None:
        with self._lock:
            self._state.update(values)
            payload = self._payload_locked()
        self._on_change(payload)

    def update_waypoint(
        self,
        index: int,
        status: str,
        message: str,
        **details: Any,
    ) -> None:
        with self._lock:
            waypoints = self._state["waypoints"]
            if 0 <= index < len(waypoints):
                waypoints[index].update(
                    {
                        "status": status,
                        "message": message,
                        **details,
                    }
                )
            payload = self._payload_locked()
        self._on_change(payload)

    def publish(self) -> None:
        with self._lock:
            payload = self._payload_locked()
        self._on_change(payload)

    def _payload_locked(self) -> str:
        return json.dumps(
            self._state,
            ensure_ascii=False,
            separators=(",", ":"),
        )
