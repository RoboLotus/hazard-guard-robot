from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ThermalCameraProfile:
    model: str
    width: int
    height: int
    frame_rate_hz: float
    horizontal_fov_deg: float
    clip_near_m: float
    visualization_range_m: float
    sensor_frame: str

    @property
    def horizontal_fov_rad(self) -> float:
        return math.radians(self.horizontal_fov_deg)


TMC160B = ThermalCameraProfile(
    model="ThermoEye TMC160B",
    width=160,
    height=120,
    frame_rate_hz=8.7,
    horizontal_fov_deg=57.0,
    clip_near_m=0.05,
    visualization_range_m=5.0,
    sensor_frame="thermal_camera_link",
)
