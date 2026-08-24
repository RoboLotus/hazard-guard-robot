from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def summarize(values: Iterable[float]) -> dict[str, float | int | None]:
    numbers = [float(value) for value in values]
    if not numbers:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p95": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(numbers),
        "mean": round(sum(numbers) / len(numbers), 6),
        "median": round(float(percentile(numbers, 0.5)), 6),
        "p95": round(float(percentile(numbers, 0.95)), 6),
        "min": round(min(numbers), 6),
        "max": round(max(numbers), 6),
    }


def angular_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class PoseSample:
    timestamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class TrajectoryAccumulator:
    minimum_step_m: float = 0.01
    maximum_step_m: float = 2.0
    samples: list[PoseSample] = field(default_factory=list)
    distance_m: float = 0.0
    dropped_jump_count: int = 0

    def add(self, sample: PoseSample) -> bool:
        if self.samples and sample.timestamp_sec < self.samples[-1].timestamp_sec:
            return False
        if self.samples:
            previous = self.samples[-1]
            distance = math.hypot(sample.x - previous.x, sample.y - previous.y)
            if distance > self.maximum_step_m:
                self.dropped_jump_count += 1
                return False
            if distance < self.minimum_step_m:
                return False
            self.distance_m += distance
        self.samples.append(sample)
        return True


@dataclass
class PhaseTimer:
    durations: dict[str, float] = field(default_factory=dict)
    _phase: str | None = None
    _timestamp: float | None = None

    def observe(self, phase: str, timestamp_sec: float) -> None:
        if self._timestamp is not None and self._phase is not None:
            elapsed = max(0.0, timestamp_sec - self._timestamp)
            self.durations[self._phase] = self.durations.get(self._phase, 0.0) + elapsed
        self._phase = phase
        self._timestamp = timestamp_sec

    def finalize(self, timestamp_sec: float) -> dict[str, float]:
        self.observe(self._phase or "unknown", timestamp_sec)
        return {
            key: round(value, 3)
            for key, value in sorted(self.durations.items())
        }


@dataclass
class LocalizationAccumulator:
    position_errors_m: list[float] = field(default_factory=list)
    yaw_errors_rad: list[float] = field(default_factory=list)

    def add(self, ground_truth: PoseSample, estimate: PoseSample) -> None:
        self.position_errors_m.append(
            math.hypot(ground_truth.x - estimate.x, ground_truth.y - estimate.y)
        )
        self.yaw_errors_rad.append(angular_error(ground_truth.yaw, estimate.yaw))

    def summary(self) -> dict[str, dict[str, float | int | None]]:
        return {
            "position_error_m": summarize(self.position_errors_m),
            "yaw_error_deg": summarize(
                math.degrees(value) for value in self.yaw_errors_rad
            ),
        }
