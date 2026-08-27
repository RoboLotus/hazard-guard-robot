import pytest

from hazard_guard_robot_telemetry.battery import (
    BatteryCalibration,
    VoltageSmoother,
)


def test_percentage_is_clamped_to_configured_voltage_range():
    calibration = BatteryCalibration(10.5, 12.6)

    assert calibration.percentage(9.0) == 0.0
    assert calibration.percentage(10.5) == 0.0
    assert calibration.percentage(11.55) == pytest.approx(0.5)
    assert calibration.percentage(12.6) == 1.0
    assert calibration.percentage(13.0) == 1.0


def test_voltage_smoother_uses_only_the_bounded_recent_window():
    smoother = VoltageSmoother(window_size=2)

    assert smoother.observe(12.0) == 12.0
    assert smoother.observe(11.8) == pytest.approx(11.9)
    assert smoother.observe(11.6) == pytest.approx(11.7)
    assert smoother.sample_count == 2


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_voltage_is_rejected(value):
    with pytest.raises(ValueError):
        VoltageSmoother().observe(value)


def test_invalid_calibration_is_rejected():
    with pytest.raises(ValueError):
        BatteryCalibration(12.6, 12.6)
