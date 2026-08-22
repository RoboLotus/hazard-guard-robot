from hazard_guard_thermal_analysis.baseline import BaselineStats, EquipmentBaseline
from hazard_guard_thermal_analysis.trend import TrendConfig, evaluate_visit


def config() -> TrendConfig:
    return TrendConfig(
        schema_version=3,
        history_window_visits=8,
        min_trend_visits=3,
        minimum_step_c=0.5,
        minimum_rise_c=1.0,
        default_adaptive_delta_c=10.0,
    )


def visit(
    temperature_c: float,
    timestamp: float,
    *,
    adaptive_threshold_enabled: bool = True,
) -> dict:
    voxel = {
        "voxel_id": "motor:0:0:0",
        "p95_temperature_c": temperature_c,
        "max_temperature_c": temperature_c,
        "point_count": 50,
    }
    return {
        "recorded_at_unix_sec": timestamp,
        "equipment": [
            {
                "equipment_id": "motor",
                "p95_valid": True,
                "statistics": {"p95_temperature_c": temperature_c},
                "thresholds": {
                    "critical_temperature_c": 49.0,
                    "adaptive_delta_c": 10.0,
                    "adaptive_threshold_enabled": adaptive_threshold_enabled,
                },
                "voxels": [voxel],
            }
        ],
    }


def environment_visit(
    temperature_c: float,
    timestamp: float,
    environment_temperature_c: float,
    point_count: int = 40,
) -> dict:
    result = visit(temperature_c, timestamp)
    result["ambient"] = {
        "median_temperature_c": environment_temperature_c,
        "point_count": point_count,
    }
    return result


def baseline(
    temperature_c: float = 30.0,
    environment_delta_c: float | None = None,
) -> dict[str, EquipmentBaseline]:
    stats = BaselineStats(
        temperature_c=temperature_c,
        sample_count=10,
        environment_delta_c=environment_delta_c,
        state="validated",
    )
    return {"motor": EquipmentBaseline("motor", stats, {})}


def decision(result: dict) -> dict:
    return result["equipment"][0]["voxels"][0]["trend_analysis"]


def test_critical_temperature_is_immediate_without_other_signals() -> None:
    current = decision(evaluate_visit(visit(50.0, 0.0), [], config()))
    assert current["status"] == "critical"
    assert current["critical"] is True


def test_trend_only_requests_recheck_without_baseline() -> None:
    history = [visit(30.0, 0.0), visit(30.5, 60.0)]
    current = decision(evaluate_visit(visit(32.0, 120.0), history, config()))
    assert current["trend"] is True
    assert current["adaptive"] is False
    assert current["status"] == "watch"
    assert current["reason"] == "trend_only_recheck"


def test_one_degree_total_rise_is_a_trend_at_the_configured_boundary() -> None:
    history = [visit(30.0, 0.0), visit(30.5, 60.0)]
    current = decision(evaluate_visit(visit(31.0, 120.0), history, config()))
    assert current["trend"] is True
    assert current["total_rise_c"] == 1.0
    assert current["minimum_rise_threshold_c"] == 1.0
    assert current["status"] == "watch"


def test_adaptive_only_requests_recheck() -> None:
    current = decision(
        evaluate_visit(
            visit(41.0, 0.0), [], config(), baselines=baseline()
        )
    )
    assert current["trend"] is False
    assert current["adaptive"] is True
    assert current["status"] == "watch"
    assert current["reason"] == "adaptive_only_recheck"


def test_trend_and_adaptive_create_warning() -> None:
    history = [visit(40.0, 0.0), visit(40.5, 60.0)]
    current = decision(
        evaluate_visit(
            visit(42.0, 120.0), history, config(), baselines=baseline()
        )
    )
    assert current["trend"] is True
    assert current["adaptive"] is True
    assert current["status"] == "warning"
    assert current["reason"] == "trend_and_adaptive"


def test_invalid_p95_cannot_trigger_any_temperature_rule() -> None:
    history = [visit(40.0, 0.0), visit(40.5, 60.0)]
    current_visit = visit(50.0, 120.0)
    current_visit["equipment"][0]["p95_valid"] = False
    current = decision(
        evaluate_visit(
            current_visit, history, config(), baselines=baseline()
        )
    )
    assert current["status"] == "normal"
    assert current["critical"] is False
    assert current["trend"] is False
    assert current["adaptive"] is False


def test_environment_compensation_cancels_uniform_temperature_shift() -> None:
    history = [
        environment_visit(30.0, 0.0, 20.0),
        environment_visit(35.0, 60.0, 25.0),
    ]
    current = decision(
        evaluate_visit(
            environment_visit(40.0, 120.0, 30.0),
            history,
            config(),
            baselines=baseline(30.0, environment_delta_c=10.0),
        )
    )
    assert current["trend"] is False
    assert current["adaptive"] is False
    assert current["status"] == "normal"
    assert current["environment_compensation_used"] is True
    assert current["trend_signal"] == "p95_minus_environment_reference"
    assert current["adaptive_signal"] == "p95_minus_environment_reference"


def test_environment_compensation_keeps_local_rise_visible() -> None:
    history = [
        environment_visit(30.0, 0.0, 20.0),
        environment_visit(30.5, 60.0, 20.0),
    ]
    current = decision(
        evaluate_visit(
            environment_visit(32.0, 120.0, 20.0),
            history,
            config(),
            baselines=baseline(20.0, environment_delta_c=0.0),
        )
    )
    assert current["trend"] is True
    assert current["adaptive"] is True
    assert current["status"] == "warning"
    assert current["compensated_temperature_c"] == 12.0


def test_insufficient_environment_points_falls_back_to_raw_values() -> None:
    history = [
        environment_visit(30.0, 0.0, 20.0),
        environment_visit(30.5, 60.0, 20.0),
    ]
    current = decision(
        evaluate_visit(
            environment_visit(32.0, 120.0, 20.0, point_count=39),
            history,
            config(),
            baselines=baseline(20.0, environment_delta_c=0.0),
        )
    )
    assert current["trend"] is True
    assert current["adaptive"] is True
    assert current["trend_signal"] == "p95_temperature"
    assert current["adaptive_signal"] == "p95_temperature"
    assert current["environment_compensation_used"] is False


def test_critical_rule_remains_absolute_with_environment_reference() -> None:
    current = decision(
        evaluate_visit(
            environment_visit(50.0, 0.0, 45.0),
            [],
            config(),
            baselines=baseline(30.0, environment_delta_c=10.0),
        )
    )
    assert current["critical"] is True
    assert current["status"] == "critical"


def test_missing_baseline_reports_collection_pending() -> None:
    current = decision(evaluate_visit(visit(30.0, 0.0), [], config()))
    assert current["status"] == "normal"
    assert current["reason"] == "baseline_pending"
    assert current["adaptive"] is False


def test_disabling_adaptive_policy_keeps_only_fixed_critical_rule() -> None:
    current = decision(
        evaluate_visit(
            visit(41.0, 0.0, adaptive_threshold_enabled=False),
            [],
            config(),
            baselines=baseline(),
        )
    )
    assert current["status"] == "normal"
    assert current["reason"] == "adaptive_disabled"
    assert current["adaptive"] is False
    assert current["adaptive_candidate"] is True
    assert current["policy_mode"] == "fixed_only"


def test_disabling_adaptive_policy_does_not_disable_absolute_critical() -> None:
    current = decision(
        evaluate_visit(
            visit(50.0, 0.0, adaptive_threshold_enabled=False),
            [],
            config(),
            baselines=baseline(),
        )
    )
    assert current["status"] == "critical"
    assert current["critical"] is True


def test_effective_threshold_tracks_environment_reference() -> None:
    current = decision(
        evaluate_visit(
            environment_visit(74.0, 0.0, 30.0),
            [],
            config(),
            baselines=baseline(60.0, environment_delta_c=35.0),
        )
    )
    assert current["effective_adaptive_threshold_c"] == 75.0
    assert current["adaptive"] is False
