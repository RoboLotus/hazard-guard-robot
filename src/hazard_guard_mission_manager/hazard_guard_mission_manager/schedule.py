from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any


REPEAT_ONCE = 0
REPEAT_COUNT = 1
REPEAT_UNTIL_TIME = 2
REPEAT_FOREVER = 3
VALID_REPEAT_MODES = {
    REPEAT_ONCE,
    REPEAT_COUNT,
    REPEAT_UNTIL_TIME,
    REPEAT_FOREVER,
}


def unix_time_ms() -> int:
    return int(time.time() * 1000)


@dataclass(frozen=True)
class PatrolSchedule:
    repeat_mode: int
    repeat_count: int
    repeat_interval_sec: float
    start_at_unix_ms: int
    end_at_unix_ms: int

    @classmethod
    def from_request(cls, request: Any) -> "PatrolSchedule":
        schedule = cls(
            repeat_mode=int(request.repeat_mode),
            repeat_count=int(request.repeat_count),
            repeat_interval_sec=float(request.repeat_interval_sec),
            start_at_unix_ms=int(request.start_at_unix_ms),
            end_at_unix_ms=int(request.end_at_unix_ms),
        )
        schedule.validate()
        return schedule

    @property
    def total_cycles(self) -> int:
        if self.repeat_mode == REPEAT_ONCE:
            return 1
        if self.repeat_mode == REPEAT_COUNT:
            return self.repeat_count
        return 0

    def validate(self) -> None:
        if self.repeat_mode not in VALID_REPEAT_MODES:
            raise ValueError("지원하지 않는 순찰 반복 방식입니다.")
        if self.repeat_mode == REPEAT_COUNT and not 2 <= self.repeat_count <= 1000:
            raise ValueError("지정 반복 횟수는 2~1000회여야 합니다.")
        if not 0.0 <= self.repeat_interval_sec <= 86400.0:
            raise ValueError("반복 대기시간은 0~86400초여야 합니다.")
        if self.start_at_unix_ms < 0 or self.end_at_unix_ms < 0:
            raise ValueError("예약 시각은 음수가 될 수 없습니다.")
        if self.repeat_mode == REPEAT_UNTIL_TIME:
            if self.end_at_unix_ms <= 0:
                raise ValueError("시각 종료 방식에는 종료 시각이 필요합니다.")
            effective_start = self.start_at_unix_ms or unix_time_ms()
            if self.end_at_unix_ms <= effective_start:
                raise ValueError("종료 시각은 시작 시각보다 뒤여야 합니다.")

    def should_continue(self, completed_cycles: int) -> bool:
        if self.repeat_mode == REPEAT_ONCE:
            return completed_cycles < 1
        if self.repeat_mode == REPEAT_COUNT:
            return completed_cycles < self.repeat_count
        return True

    def deadline_reached(self, now_ms: int | None = None) -> bool:
        return (
            self.repeat_mode == REPEAT_UNTIL_TIME
            and self.end_at_unix_ms > 0
            and (now_ms if now_ms is not None else unix_time_ms())
            >= self.end_at_unix_ms
        )
