from __future__ import annotations

import threading


class SafetyPauseLatch:
    """Thread-safe mission pause state independent from ROS message classes."""

    STOP = 3
    SENSOR_FAULT = 4

    def __init__(self, *, enabled: bool = False) -> None:
        self._enabled = enabled
        self._paused = False
        self._state = 0
        self._reason = ""
        self._lock = threading.RLock()

    def update(self, state: int, reason: str = "") -> bool:
        """Update the state and return True only when pause status changes."""
        with self._lock:
            previous = self._paused
            self._state = int(state)
            self._reason = str(reason)
            self._paused = self._enabled and self._state in {
                self.STOP,
                self.SENSOR_FAULT,
            }
            return previous != self._paused

    def is_paused(self) -> bool:
        with self._lock:
            return self._paused

    def snapshot(self) -> tuple[int, str]:
        with self._lock:
            return self._state, self._reason
