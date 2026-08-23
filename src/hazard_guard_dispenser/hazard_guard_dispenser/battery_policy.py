from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BatteryPolicy:
    empty_voltage: float = 9.0
    full_voltage: float = 12.6
    low_voltage: float = 10.5
    critical_voltage: float = 10.0
    valid_min_voltage: float = 7.5
    valid_max_voltage: float = 13.5

    def __post_init__(self) -> None:
        values = (
            self.valid_min_voltage,
            self.empty_voltage,
            self.critical_voltage,
            self.low_voltage,
            self.full_voltage,
            self.valid_max_voltage,
        )
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("배터리 전압 기준은 모두 0보다 커야 합니다")
        if values != tuple(sorted(values)):
            raise ValueError(
                "배터리 전압 기준은 valid_min <= empty <= critical <= "
                "low <= full <= valid_max 순서여야 합니다"
            )
        if self.full_voltage == self.empty_voltage:
            raise ValueError("배터리 empty와 full 전압은 달라야 합니다")

    def percent(self, voltage: float) -> int:
        ratio = (float(voltage) - self.empty_voltage) / (
            self.full_voltage - self.empty_voltage
        )
        return int(max(0.0, min(1.0, ratio)) * 100.0 + 0.5)

    def state(self, voltage: float | None, *, stale: bool = False) -> str:
        if voltage is None or stale:
            return "unknown"
        value = float(voltage)
        if value < self.valid_min_voltage or value > self.valid_max_voltage:
            return "invalid"
        if value <= self.critical_voltage:
            return "critical"
        if value <= self.low_voltage:
            return "low"
        return "normal"

    def available_for_drop(
        self,
        voltage: float | None,
        *,
        connected: bool,
        stale: bool = False,
        allow_unknown: bool = False,
    ) -> bool:
        if not connected:
            return False
        state = self.state(voltage, stale=stale)
        if state == "unknown":
            return bool(allow_unknown and voltage is None and not stale)
        return state not in {"critical", "invalid"}
