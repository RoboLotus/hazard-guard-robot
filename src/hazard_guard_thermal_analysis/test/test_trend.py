import json

from hazard_guard_thermal_analysis.trend import (
    TrendConfig,
    evaluate_visit,
    read_history,
)


def make_visit(temperature, ambient=25.0, *, critical=80.0):
    return {
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
                        "delta_p95_c": temperature - ambient,
                        "point_count": 10,
                        "center": [0.1, 0.1, 0.1],
                    }
                ],
            }
        ]
    }


def decision(result):
    return result["equipment"][0]["voxels"][0]["trend_analysis"]


def test_critical_temperature_is_immediate():
    result = evaluate_visit(make_visit(82.0), [], TrendConfig())

    assert decision(result)["status"] == "critical"
    assert decision(result)["critical"] is True


def test_persistent_ambient_corrected_rise_and_anomaly_become_warning():
    config = TrendConfig(
        minimum_rise_c=2.0,
        minimum_slope_c_per_visit=0.75,
        adaptive_residual_c=1.0,
    )
    history = [make_visit(38.0, 25.0), make_visit(40.0, 25.0)]

    result = evaluate_visit(make_visit(43.0, 25.0), history, config)

    current = decision(result)
    assert current["signal"] == "ambient_corrected_p95"
    assert current["trend"] is True
    assert current["adaptive"] is True
    assert current["status"] == "warning"
    assert result["equipment"][0]["trend_status"] == "warning"


def test_trend_without_threshold_is_watch_not_warning():
    config = TrendConfig(minimum_rise_c=2.0, minimum_slope_c_per_visit=0.75)
    history = [make_visit(30.0), make_visit(32.0)]

    result = evaluate_visit(make_visit(34.0), history, config)

    current = decision(result)
    assert current["trend"] is True
    assert current["adaptive"] is False
    assert current["status"] == "watch"


def test_history_reader_skips_partial_lines(tmp_path):
    path = tmp_path / "history.jsonl"
    path.write_text(
        json.dumps(make_visit(30.0)) + "\n{partial\n" + json.dumps(make_visit(32.0)) + "\n",
        encoding="utf-8",
    )

    history = read_history(path)

    assert len(history) == 2