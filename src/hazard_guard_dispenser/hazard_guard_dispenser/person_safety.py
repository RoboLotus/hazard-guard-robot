from __future__ import annotations

import math
import threading
import time


class PersonSafetyLatch:
    """Fail-closed freshness check for physical dispenser actuation."""

    CLEAR = 0

    def __init__(self, *, required: bool, timeout_sec: float) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("person_safety_timeout_sec는 0보다 커야 합니다")
        self._required = bool(required)
        self._timeout_sec = float(timeout_sec)
        self._lock = threading.Lock()
        self._state: int | None = None
        self._detector_stale = True
        self._updated_monotonic: float | None = None

    def update(
        self,
        *,
        state: int,
        detector_stale: bool,
        now_monotonic: float | None = None,
    ) -> None:
        with self._lock:
            self._state = int(state)
            self._detector_stale = bool(detector_stale)
            self._updated_monotonic = (
                time.monotonic()
                if now_monotonic is None
                else float(now_monotonic)
            )

    def block_reason(self, *, now_monotonic: float | None = None) -> str | None:
        if not self._required:
            return None
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        with self._lock:
            state = self._state
            detector_stale = self._detector_stale
            updated = self._updated_monotonic
        if updated is None:
            return "person_safety_unknown"
        if now - updated > self._timeout_sec:
            return "person_safety_stale"
        if detector_stale:
            return "person_detector_stale"
        if state != self.CLEAR:
            return "person_safety_not_clear"
        return None
