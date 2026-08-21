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


def test_co_can_raise_warning_and_critical_independently() -> None:
    scenario = GasScenario.load(CONFIG)
    engine = GasDecisionEngine(scenario.ambient, scenario.decision)
    warning = engine.update(GasVector(100.0, 6.0, 500.0), 10.0, True)
    assert warning.state == "warning"
    critical = engine.update(GasVector(100.0, 25.0, 900.0), 11.0, True)
    assert critical.state == "critical"
