from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BatteryCalibration:
    """Configurable reference conversion for the ROSMASTER 3S battery."""

    empty_voltage_v: float = 10.5
    full_voltage_v: float = 12.6

    def __post_init__(self) -> None:
        if not math.isfinite(self.empty_voltage_v):
            raise ValueError("empty_voltage_v must be finite")
        if not math.isfinite(self.full_voltage_v):
            raise ValueError("full_voltage_v must be finite")
        if self.full_voltage_v <= self.empty_voltage_v:
            raise ValueError("full_voltage_v must be greater than empty_voltage_v")

    def percentage(self, voltage_v: float) -> float:
        if not math.isfinite(voltage_v) or voltage_v <= 0.0:
            raise ValueError("battery voltage must be a positive finite number")
        ratio = (voltage_v - self.empty_voltage_v) / (
            self.full_voltage_v - self.empty_voltage_v
        )
        return min(1.0, max(0.0, ratio))


class VoltageSmoother:
    """Bounded moving average that rejects invalid board readings."""

    def __init__(self, window_size: int = 20) -> None:
        if isinstance(window_size, bool) or int(window_size) < 1:
            raise ValueError("window_size must be at least 1")
        self._samples: deque[float] = deque(maxlen=int(window_size))

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def observe(self, voltage_v: float) -> float:
        value = float(voltage_v)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("battery voltage must be a positive finite number")
        self._samples.append(value)
        return sum(self._samples) / len(self._samples)
