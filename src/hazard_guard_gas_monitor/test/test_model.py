from pathlib import Path

from hazard_guard_gas_monitor.model import (
    FirstOrderGasSensor,
    FirstOrderThermalSource,
    GasScenario,
)


CONFIG = Path(__file__).parents[1] / "config" / "demo_gas_scenario.json"


def test_timeline_enters_voc_before_heating_and_combustion() -> None:
    scenario = GasScenario.load(CONFIG)
    assert scenario.source.visual_model_id == "discarded_lithium_power_bank"
    assert scenario.source.x == -1.70464
    assert scenario.source.y == -0.718916
    assert scenario.phase_at(14.9).phase == "normal"
    assert scenario.phase_at(15.0).phase == "voc_venting"
    assert scenario.phase_at(55.0).phase == "heating"
    assert scenario.phase_at(95.0).phase == "combustion"
    assert scenario.phase_at(15.0).surface_temperature_c < 30.0
    assert scenario.phase_at(55.0).surface_temperature_c == 58.0
    assert scenario.phase_at(95.0).surface_temperature_c == 105.0
    assert scenario.decision.voc_absolute_watch_index == 150.0
    assert scenario.decision.co_absolute_warning_ppm == 30.0
    assert scenario.decision.co_absolute_critical_ppm == 200.0
    assert scenario.decision.baseline_min_visits == 6


def test_fan_reduces_sensor_lag() -> None:
    scenario = GasScenario.load(CONFIG)
    target = scenario.ambient.add(scenario.timeline[1].emission)
    passive = FirstOrderGasSensor(scenario.ambient, scenario.sensor)
    active = FirstOrderGasSensor(scenario.ambient, scenario.sensor)
    passive_value = passive.update(target, 1.0, False)
    active_value = active.update(target, 1.0, True)
    assert active_value.voc_index > passive_value.voc_index


def test_surface_temperature_rises_smoothly_toward_phase_target() -> None:
    source = FirstOrderThermalSource(25.0, 10.0)
    first = source.update(105.0, 1.0)
    second = source.update(105.0, 1.0)
    assert 25.0 < first < second < 105.0
