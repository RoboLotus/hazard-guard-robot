"""Linearizable cancellation gate for one physical dispenser actuation."""

from __future__ import annotations

import threading
from collections.abc import Callable


class ActuationGate:
    """Make pre-actuation cancellation and the physical start mutually exclusive."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._cancel_requested: set[str] = set()
        self._started: set[str] = set()

    def request_cancel(
        self,
        request_id: str,
        *,
        eligible: Callable[[], bool] | None = None,
    ) -> bool:
        with self._lock:
            if request_id in self._started:
                return False
            if eligible is not None and not eligible():
                return False
            self._cancel_requested.add(request_id)
            return True

    def cancel_requested(self, request_id: str) -> bool:
        with self._lock:
            return request_id in self._cancel_requested

    def claim_actuation(
        self,
        request_id: str,
        *,
        mark_started: Callable[[], None],
    ) -> bool:
        """Commit the durable start marker while cancellation is excluded."""

        with self._lock:
            if request_id in self._cancel_requested:
                return False
            mark_started()
            self._started.add(request_id)
            return True

    def finish(self, request_id: str) -> None:
        with self._lock:
            self._cancel_requested.discard(request_id)
            self._started.discard(request_id)
