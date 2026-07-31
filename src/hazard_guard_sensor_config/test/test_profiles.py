import math

from hazard_guard_sensor_config import TMC160B


def test_tmc160b_profile_is_internally_consistent() -> None:
    assert TMC160B.model == "ThermoEye TMC160B"
    assert (TMC160B.width, TMC160B.height) == (160, 120)
    assert math.isclose(TMC160B.horizontal_fov_rad, math.radians(57.0))
    assert TMC160B.clip_near_m < TMC160B.visualization_range_m
