import json

from hazard_guard_thermal_analysis.trend import (
    TrendConfig,
    evaluate_visit,
    read_history,
)


def make_visit(
    temperature,
    ambient=25.0,
    *,
    critical=80.0,
    max_temperature=None,
    recorded_at=None,
):
    return {
        "recorded_at_unix_sec": recorded_at,
        "equipment": [
            {
                "equipment_id": "motor",
                "thresholds": {
                    "warning_temperature_c": 45.0,
                    "critical_temperature_c": critical,
                    "warning_delta_c": 15.0,
                },
                "voxels": [
                    {
                        "voxel_id": "motor:0:0:0",
                        "p95_temperature_c": temperature,
                        "max_temperature_c": (
                            temperature
                            if max_temperature is None
                            else max_temperature
                        ),
                        "delta_p95_c": temperature - ambient,
                        "point_count": 10,
                        "center": [0.1, 0.1, 0.1],
                    }
                ],
            }
        ],
    }


def decision(result):
    return result["equipment"][0]["voxels"][0]["trend_analysis"]


def test_critical_p95_temperature_is_immediate():
    result = evaluate_visit(make_visit(82.0), [], TrendConfig())

    current = decision(result)
    assert current["status"] == "critical"
    assert current["critical"] is True
    assert current["critical_p95"] is True
    assert current["critical_max"] is True


def test_max_temperature_can_trigger_immediate_critical_when_p95_is_lower():
    result = evaluate_visit(
        make_visit(62.0, max_temperature=82.0), [], TrendConfig()
    )

    current = decision(result)
    assert current["status"] == "critical"
    assert current["critical_p95"] is False
    assert current["critical_max"] is True
    assert current["reason"] == "critical_max_temperature"


def test_persistent_time_based_rise_and_anomaly_become_warning():
    config = TrendConfig(
        minimum_rise_c=2.0,
        minimum_slope_c_per_hour=2.0,
        adaptive_residual_c=1.0,
    )
    history = [
        make_visit(38.0, 25.0, recorded_at=0.0),
        make_visit(40.0, 25.0, recorded_at=3600.0),
    ]

    result = evaluate_visit(
        make_visit(43.0, 25.0, recorded_at=7200.0), history, config
    )

    current = decision(result)
    assert current["signal"] == "ambient_corrected_p95"
    assert current["trend"] is True
    assert current["adaptive"] is True
    assert current["status"] == "warning"
    assert current["slope_c_per_hour"] == 2.5
    assert current["time_span_hours"] == 2.0
    assert result["equipment"][0]["trend_status"] == "warning"


def test_time_based_trend_without_threshold_is_watch_not_warning():
    config = TrendConfig(
        minimum_rise_c=2.0,
        minimum_slope_c_per_hour=2.0,
    )
    history = [
        make_visit(30.0, recorded_at=0.0),
        make_visit(32.0, recorded_at=3600.0),
    ]

    result = evaluate_visit(
        make_visit(34.0, recorded_at=7200.0), history, config
    )

    current = decision(result)
    assert current["trend"] is True
    assert current["adaptive"] is False
    assert current["status"] == "watch"


def test_same_temperature_rise_over_longer_time_does_not_meet_rate():
    config = TrendConfig(
        minimum_rise_c=2.0,
        minimum_slope_c_per_hour=2.0,
    )
    history = [
        make_visit(30.0, recorded_at=0.0),
        make_visit(32.0, recorded_at=7200.0),
    ]

    result = evaluate_visit(
        make_visit(34.0, recorded_at=14400.0), history, config
    )

    current = decision(result)
    assert current["total_rise_c"] == 4.0
    assert current["slope_c_per_hour"] == 1.0
    assert current["trend"] is False


def test_mixed_clock_sources_do_not_create_a_false_trend():
    first = make_visit(30.0, recorded_at=0.0)
    second = make_visit(32.0)
    second["stamp"] = {"sec": 3600, "nanosec": 0}

    result = evaluate_visit(
        make_visit(36.0, recorded_at=7200.0),
        [first, second],
        TrendConfig(),
    )

    assert decision(result)["trend"] is False


def test_history_reader_skips_partial_lines(tmp_path):
    path = tmp_path / "history.jsonl"
    path.write_text(
        json.dumps(make_visit(30.0))
        + "\n{partial\n"
        + json.dumps(make_visit(32.0))
        + "\n",
        encoding="utf-8",
    )

    history = read_history(path)

    assert len(history) == 2
