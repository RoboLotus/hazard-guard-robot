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
    # Datasheet figures the simulation does not model yet. They are here so the
    # numbers used to judge a detection have one source: a reading is only
    # meaningful against the sensor's own accuracy and measuring range.
    netd_mk: float
    accuracy_c: float
    measure_min_c: float
    measure_max_c: float

    @property
    def horizontal_fov_rad(self) -> float:
        return math.radians(self.horizontal_fov_deg)


# Manufacturer specification table for TMC160BB / 160BH, read off the product
# page. The sensor is given only as an uncooled VOx microbolometer, 12 um
# pitch, 8-14 um - the vendor part number is not published.
#
# The lens ships in two versions, 57 and 95 degrees. This is the 57 one.
#
# Measuring range is the high-gain setting: -10 to 140 C, accurate to +-5 C or
# +-5%. Low gain reaches 450 C at +-10 C, which is the setting an actual fire
# would need - the profile below is the high-gain one the demo hazards
# (48-85 C) live in.
TMC160B = ThermalCameraProfile(
    model="ThermoEye TMC160B",
    width=160,
    height=120,
    frame_rate_hz=8.7,
    horizontal_fov_deg=57.0,
    clip_near_m=0.05,
    visualization_range_m=5.0,
    sensor_frame="thermal_camera_link",
    netd_mk=50.0,
    accuracy_c=5.0,
    measure_min_c=-10.0,
    measure_max_c=140.0,
)
