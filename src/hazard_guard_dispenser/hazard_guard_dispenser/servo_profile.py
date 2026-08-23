from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoProfile:
    home_angle: int
    dump_angle: int
    minimum_angle: int
    maximum_angle: int
    step_deg: int
    step_delay_sec: float

    def __post_init__(self) -> None:
        if not 0 <= self.minimum_angle < self.maximum_angle <= 180:
            raise ValueError("서보 안전 각도는 0 <= min < max <= 180이어야 합니다")
        self.validate_target(self.home_angle, label="home")
        self.validate_target(self.dump_angle, label="dump")
        if self.home_angle == self.dump_angle:
            raise ValueError("서보 home과 dump 각도는 달라야 합니다")
        if self.step_deg <= 0:
            raise ValueError("서보 step_deg는 0보다 커야 합니다")
        if self.step_delay_sec < 0:
            raise ValueError("서보 step_delay는 0 이상이어야 합니다")

    def validate_target(self, angle: int, *, label: str = "target") -> int:
        value = int(angle)
        if not self.minimum_angle <= value <= self.maximum_angle:
            raise ValueError(
                f"서보 {label} 각도 {value}가 안전 범위 "
                f"{self.minimum_angle}..{self.maximum_angle} 밖입니다"
            )
        return value
