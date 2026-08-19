from hazard_guard_performance_monitor.statistics import (
    percentile,
    summarize_values,
)


def test_percentile_uses_linear_interpolation():
    assert percentile([0, 10, 20, 30, 40], 0.95) == 38.0


def test_summary_ignores_non_finite_values():
    summary = summarize_values([1.0, 2.0, float("nan"), 3.0])

    assert summary["count"] == 3
    assert summary["mean"] == 2.0
    assert summary["median"] == 2.0
    assert summary["p95"] == 2.9
    assert summary["max"] == 3.0


def test_empty_summary_is_explicit():
    assert summarize_values([]) == {
        "count": 0,
        "mean": None,
        "median": None,
        "p95": None,
        "max": None,
        "stddev": None,
    }
