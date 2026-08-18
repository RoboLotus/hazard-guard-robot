import json

from hazard_guard_thermal_analysis.baseline import BaselineStats, EquipmentBaseline
from hazard_guard_thermal_analysis.trend import TrendConfig, evaluate_visit, read_history


def thresholds(**overrides):
    values = {
        "threshold_mode": "absolute",
        "watch_temperature_c": 40.0,
        "warning_temperature_c": 50.0,
        "critical_temperature_c": 65.0,
        "watch_delta_c": 10.0,
        "warning_delta_c": 20.0,
        "critical_delta_c": None,
        "simulation_fallback_temperature_c": {"watch": None, "warning": None, "critical": None},
        "baseline_delta_c": {"watch": None, "warning": None, "critical": None},
        "air_delta_c": {"watch": None, "warning": None, "critical": None},
        "oil_temperature_c": {"watch": None, "warning": None, "critical": None},
        "surface_alone_can_trip": True,
    }
    values.update(overrides)
    return values


def make_visit(
    temperature,
    *,
    equipment_id="waste",
    peak=None,
    recorded_at=None,
    threshold_values=None,
    p95_valid=True,
    cluster=0,
    index=(0, 0, 0),
    reference_delta=None,
):
    peak = temperature if peak is None else peak
    voxel = {
        "voxel_id": f"{equipment_id}:{index[0]}:{index[1]}:{index[2]}",
        "index": list(index),
        "center": [0.1, 0.1, 0.1],
        "mean_temperature_c": temperature - 1.0,
        "median_temperature_c": temperature - 0.5,
        "p90_temperature_c": temperature - 0.1,
        "p95_temperature_c": temperature,
        "max_temperature_c": peak,
        "point_count": 50,
        "max_hot_cluster_pixels": cluster,
        "reference_delta_p95_c": reference_delta,
    }
    return {
        "schema_version": 2,
        "recorded_at_unix_sec": recorded_at,
        "equipment": [{
            "equipment_id": equipment_id,
            "p95_valid": p95_valid,
            "reference_delta_p95_c": reference_delta,
            "statistics": {
                "median_temperature_c": temperature - 0.5,
                "p95_temperature_c": temperature,
                "max_temperature_c": peak,
                "point_count": 50,
            },
            "thresholds": threshold_values or thresholds(),
            "voxels": [voxel],
        }],
    }


def decision(result):
    return result["equipment"][0]["voxels"][0]["trend_analysis"]


def baseline(equipment_id="motor", *, sigma_normal=1.0, sigma_repeat=0.2, sigma_residual=1.0):
    stats = BaselineStats(
        temperature_c=30.0,
        sigma_normal_c=sigma_normal,
        sigma_repeat_c=sigma_repeat,
        sigma_residual_c=sigma_residual,
        sample_count=20,
        state="validated",
    )
    return {equipment_id: EquipmentBaseline(equipment_id, stats, {})}


def test_p95_critical_is_immediate_when_sample_count_is_valid():
    result = evaluate_visit(make_visit(66.0), [], TrendConfig(), simulated=False)
    current = decision(result)
    assert current["status"] == "critical"
    assert current["critical_p95"] is True
    assert current["critical_max"] is False


def test_single_max_candidate_requires_confirmation():
    result = evaluate_visit(
        make_visit(55.0, peak=70.0, cluster=1), [], TrendConfig(), simulated=False
    )
    current = decision(result)
    assert current["status"] == "warning"
    assert current["max_candidate"] is True
    assert current["critical_max"] is False
    assert current["reason"] == "unconfirmed_single_max_candidate"


def test_max_with_three_by_three_radiometric_cluster_is_critical():
    result = evaluate_visit(
        make_visit(55.0, peak=70.0, cluster=9), [], TrendConfig(), simulated=False
    )
    current = decision(result)
    assert current["status"] == "critical"
    assert current["critical_max"] is True
    assert current["max_spatial_cluster_pixels"] == 9


def test_p95_is_ignored_when_roi_sample_count_is_insufficient():
    result = evaluate_visit(
        make_visit(70.0, peak=60.0, p95_valid=False), [], TrendConfig(), simulated=False
    )
    assert decision(result)["status"] == "normal"


def test_same_material_reference_delta_drives_waste_warning():
    result = evaluate_visit(
        make_visit(48.0, reference_delta=22.0), [], TrendConfig(), simulated=False
    )
    current = decision(result)
    assert current["status"] == "warning"
    assert current["reason"] == "warning_same_material_reference_delta"


def test_motor_uses_approved_baseline_in_production_and_simulation_fallback_in_sim():
    motor_thresholds = thresholds(
        threshold_mode="baseline_primary",
        watch_temperature_c=None,
        warning_temperature_c=None,
        critical_temperature_c=None,
        simulation_fallback_temperature_c={"watch": 70.0, "warning": 80.0, "critical": 90.0},
        baseline_delta_c={"watch": 10.0, "warning": 15.0, "critical": 20.0},
    )
    production = evaluate_visit(
        make_visit(46.0, equipment_id="motor", threshold_values=motor_thresholds),
        [], TrendConfig(), baselines=baseline(), simulated=False,
    )
    critical_candidate = evaluate_visit(
        make_visit(51.0, equipment_id="motor", threshold_values=motor_thresholds),
        [], TrendConfig(), baselines=baseline(), simulated=False,
    )
    simulation = evaluate_visit(
        make_visit(81.0, equipment_id="motor", threshold_values=motor_thresholds),
        [], TrendConfig(), simulated=True,
    )
    assert decision(production)["status"] == "warning"
    assert decision(production)["reason"] == "warning_approved_baseline_delta"
    assert decision(critical_candidate)["status"] == "warning"
    assert decision(critical_candidate)["reason"] == "baseline_critical_candidate_requires_corroboration"
    assert decision(simulation)["status"] == "warning"
    assert decision(simulation)["reason"] == "warning_p95_temperature"


def test_tank_surface_critical_is_screening_until_oil_sensor_confirms():
    tank_thresholds = thresholds(
        oil_temperature_c={"watch": 50.0, "warning": 60.0, "critical": 70.0},
        surface_alone_can_trip=False,
    )
    visit = make_visit(66.0, equipment_id="tank", threshold_values=tank_thresholds)
    surface_only = evaluate_visit(visit, [], TrendConfig(), simulated=False)
    confirmed = evaluate_visit(
        visit, [], TrendConfig(), sensor_values={"oil_temperature_c": 71.0}, simulated=False
    )
    assert decision(surface_only)["status"] == "warning"
    assert decision(surface_only)["reason"] == "surface_screening_requires_direct_sensor_confirmation"
    assert decision(confirmed)["status"] == "critical"


def test_six_monotonic_patrol_values_use_baseline_sigma_and_never_become_critical():
    config = TrendConfig()
    history = [
        make_visit(value, equipment_id="motor", recorded_at=index * 3600.0,
                   threshold_values=thresholds(watch_temperature_c=None, warning_temperature_c=None, critical_temperature_c=None))
        for index, value in enumerate((30.5, 31.5, 32.5, 33.5, 34.5))
    ]
    current = make_visit(
        35.5, equipment_id="motor", recorded_at=5 * 3600.0,
        threshold_values=thresholds(watch_temperature_c=None, warning_temperature_c=None, critical_temperature_c=None),
    )
    result = evaluate_visit(current, history, config, baselines=baseline(), simulated=True)
    current_decision = decision(result)
    assert current_decision["trend"] is True
    assert current_decision["status"] == "warning"
    assert current_decision["visit_count"] == 6
    assert current_decision["noise_deadband_c"] == 0.6
    assert current_decision["minimum_rise_threshold_c"] == 3.0
    assert current_decision["critical"] is False


def test_trend_is_disabled_without_an_approved_baseline():
    history = [make_visit(30.0 + index, recorded_at=index * 3600.0) for index in range(5)]
    result = evaluate_visit(make_visit(35.0, recorded_at=5 * 3600.0), history, TrendConfig())
    assert decision(result)["trend"] is False
    assert decision(result)["baseline_state"] == "missing"


def test_mixed_clock_sources_do_not_create_a_false_trend():
    first = make_visit(30.0, equipment_id="motor", recorded_at=0.0)
    second = make_visit(31.0, equipment_id="motor")
    second["stamp"] = {"sec": 3600, "nanosec": 0}
    history = [first, second] + [make_visit(32.0 + index, equipment_id="motor", recorded_at=(index + 2) * 3600.0) for index in range(3)]
    result = evaluate_visit(
        make_visit(36.0, equipment_id="motor", recorded_at=6 * 3600.0),
        history, TrendConfig(), baselines=baseline(),
    )
    assert decision(result)["trend"] is False


def test_history_reader_skips_partial_lines(tmp_path):
    path = tmp_path / "history.jsonl"
    path.write_text(json.dumps(make_visit(30.0)) + "\n{partial\n" + json.dumps(make_visit(32.0)) + "\n", encoding="utf-8")
    assert len(read_history(path)) == 2


def test_visit_index_keeps_increasing_after_history_window_and_restart(tmp_path):
    config = TrendConfig()
    history = []
    completed = []
    for visit_number in range(1, 11):
        result = evaluate_visit(make_visit(30.0, recorded_at=visit_number * 60.0), history, config)
        assert result["trend_analysis"]["visit_index"] == visit_number
        completed.append(result)
        history = (history + [result])[-config.history_window_visits:]
    path = tmp_path / "history.jsonl"
    path.write_text("".join(json.dumps(item) + "\n" for item in completed), encoding="utf-8")
    restarted = read_history(path, config.history_window_visits)
    next_result = evaluate_visit(make_visit(30.0, recorded_at=11 * 60.0), restarted, config)
    assert len(restarted) == config.history_window_visits
    assert next_result["trend_analysis"]["visit_index"] == 11
