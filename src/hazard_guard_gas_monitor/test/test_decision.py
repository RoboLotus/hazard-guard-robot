from pathlib import Path

from hazard_guard_gas_monitor.decision import GasDecisionEngine
from hazard_guard_gas_monitor.model import GasScenario, GasVector


CONFIG = Path(__file__).parents[1] / "config" / "demo_gas_scenario.json"


def test_voc_only_stops_for_investigation_instead_of_being_ignored() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(scenario.ambient, scenario.decision)
    high_voc = GasVector(150.0, 0.3, 450.0)
    assert engine.update(high_voc, 10.0, True).state == "normal"
    assert engine.update(high_voc, 10.5, True).state == "normal"
    watch = engine.update(high_voc, 11.0, True)
    assert watch.state == "voc_watch"
    assert watch.navigation_pause is True
    investigating = engine.update(high_voc, 20.0, True)
    assert investigating.state == "investigating"
    assert investigating.navigation_pause is False
    assert investigating.fan_on is True


def test_co_baseline_excursion_raises_warning() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(scenario.ambient, scenario.decision)
    warning = engine.update(GasVector(100.0, 6.0, 500.0), 10.0, True)
    assert warning.state == "warning"
    assert warning.reason == "co_baseline_excursion"


def test_absolute_co_thresholds_do_not_depend_on_baseline() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(GasVector(130.0, 28.0, 700.0), scenario.decision)
    warning = engine.update(GasVector(130.0, 30.0, 700.0), 10.0, True)
    assert warning.state == "warning"
    assert warning.reason == "co_absolute_warning"
    critical = engine.update(GasVector(130.0, 200.0, 900.0), 11.0, True)
    assert critical.state == "critical"
    assert critical.reason == "co_absolute_critical"


def test_relative_checks_wait_for_a_measured_baseline() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(
        scenario.ambient,
        scenario.decision,
        baseline_ready=False,
    )
    assert engine.update(GasVector(149.0, 6.0, 450.0), 10.0, True).state == "normal"
    assert engine.update(GasVector(149.0, 6.0, 450.0), 11.0, True).state == "normal"
    assert engine.update(GasVector(149.0, 6.0, 450.0), 12.0, True).state == "normal"
    assert engine.update(GasVector(100.0, 30.0, 450.0), 13.0, True).state == "warning"


def test_voc_fixed_action_threshold_survives_a_high_learned_baseline() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(GasVector(120.0, 0.3, 450.0), scenario.decision)
    reading = GasVector(150.0, 0.3, 450.0)
    engine.update(reading, 10.0, True)
    engine.update(reading, 11.0, True)
    assert engine.update(reading, 12.0, True).state == "voc_watch"


def test_absolute_voc_watch_can_clear_before_baseline_is_ready() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(
        scenario.ambient,
        scenario.decision,
        baseline_ready=False,
    )
    high_voc = GasVector(150.0, 0.3, 450.0)
    engine.update(high_voc, 10.0, True)
    engine.update(high_voc, 11.0, True)
    assert engine.update(high_voc, 12.0, True).state == "voc_watch"

    recovered = GasVector(149.0, 0.3, 450.0)
    for elapsed in (13.0, 14.0, 15.0, 16.0):
        assert engine.update(recovered, elapsed, True).state == "voc_watch"
    assert engine.update(recovered, 17.0, True).state == "normal"
