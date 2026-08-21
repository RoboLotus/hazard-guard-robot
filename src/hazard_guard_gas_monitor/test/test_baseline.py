from hazard_guard_gas_monitor.baseline import GasBaselineStore
from hazard_guard_gas_monitor.model import GasVector


def visit(voc: float, co: float, co2: float) -> list[GasVector]:
    return [
        GasVector(voc - 2.0, co - 0.1, co2 - 5.0),
        GasVector(voc, co, co2),
        GasVector(voc + 3.0, co + 0.2, co2 + 8.0),
    ]


def test_six_normal_visits_create_component_wise_median(tmp_path) -> None:
    store = GasBaselineStore(tmp_path / "gas.json", min_visits=6)
    medians = [
        (98.0, 0.2, 440.0),
        (100.0, 0.4, 450.0),
        (102.0, 0.3, 460.0),
        (101.0, 0.5, 455.0),
        (99.0, 0.2, 445.0),
        (103.0, 0.6, 465.0),
    ]
    for index, values in enumerate(medians):
        baseline = store.add_visit("pump_1", False, visit(*values))
        if index < 5:
            assert baseline is None

    count, baseline = store.status("pump_1", False)
    assert count == 6
    assert baseline is not None
    assert baseline.voc_index == 100.5
    assert baseline.co_ppm == 0.35
    assert baseline.co2_ppm == 452.5


def test_baseline_is_frozen_and_modes_are_separated(tmp_path) -> None:
    path = tmp_path / "gas.json"
    store = GasBaselineStore(path, min_visits=2)
    store.add_visit("pump_1", False, visit(100.0, 0.3, 450.0))
    robot = store.add_visit("pump_1", False, visit(104.0, 0.5, 470.0))
    assert robot is not None
    frozen = store.add_visit("pump_1", False, visit(400.0, 100.0, 5000.0))
    assert frozen == robot

    assert store.status("pump_1", True) == (0, None)
    reloaded = GasBaselineStore(path, min_visits=2)
    assert reloaded.status("pump_1", False)[1] == robot


def test_malformed_persisted_samples_are_ignored(tmp_path) -> None:
    path = tmp_path / "gas.json"
    path.write_text(
        '{"records":{"robot:pump_1":{"samples":[null,'
        '{"voc_index":"bad","co_ppm":0.2,"co2_ppm":450.0}],'
        '"baseline":null}}}',
        encoding="utf-8",
    )
    store = GasBaselineStore(path, min_visits=2)
    assert store.status("pump_1", False) == (0, None)
    assert store.add_visit(
        "pump_1",
        False,
        visit(100.0, 0.3, 450.0),
    ) is None
    baseline = store.add_visit(
        "pump_1",
        False,
        visit(102.0, 0.5, 470.0),
    )
    assert baseline is not None
    assert baseline.visit_count == 2


def test_increased_visit_requirement_resumes_collection(tmp_path) -> None:
    path = tmp_path / "gas.json"
    original = GasBaselineStore(path, min_visits=2)
    original.add_visit("pump_1", False, visit(100.0, 0.3, 450.0))
    assert original.add_visit(
        "pump_1",
        False,
        visit(102.0, 0.5, 470.0),
    ) is not None

    stricter = GasBaselineStore(path, min_visits=3)
    assert stricter.status("pump_1", False) == (2, None)
    baseline = stricter.add_visit(
        "pump_1",
        False,
        visit(104.0, 0.7, 490.0),
    )
    assert baseline is not None
    assert baseline.visit_count == 3
    assert baseline.voc_index == 102.0
