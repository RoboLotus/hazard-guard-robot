from pathlib import Path

from hazard_guard_thermal_analysis.trend import load_trend_config


def test_facility_config_enables_environment_reference_compensation() -> None:
    path = (
        Path(__file__).parents[1]
        / "config"
        / "demo_facility_scaled_rois.json"
    )
    config = load_trend_config(path)
    assert (config.environment_reference_enabled, config.minimum_environment_points) == (True, 40)
