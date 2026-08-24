import math

import pytest

from hazard_guard_patrol_benchmark.metrics import (
    LocalizationAccumulator,
    PhaseTimer,
    PoseSample,
    TrajectoryAccumulator,
    summarize,
)


def test_trajectory_filters_jitter_and_large_jump() -> None:
    trajectory = TrajectoryAccumulator(minimum_step_m=0.1, maximum_step_m=2.0)
    assert trajectory.add(PoseSample(0.0, 0.0, 0.0, 0.0))
    assert not trajectory.add(PoseSample(1.0, 0.01, 0.0, 0.0))
    assert trajectory.add(PoseSample(2.0, 1.0, 0.0, 0.0))
    assert not trajectory.add(PoseSample(3.0, 9.0, 0.0, 0.0))
    assert trajectory.distance_m == pytest.approx(1.0)
    assert trajectory.dropped_jump_count == 1


def test_phase_timer_splits_simulation_time() -> None:
    timer = PhaseTimer()
    timer.observe("executing", 10.0)
    timer.observe("dwelling", 13.0)
    assert timer.finalize(15.0) == {"dwelling": 2.0, "executing": 3.0}


def test_localization_summary_uses_position_and_shortest_angle() -> None:
    accumulator = LocalizationAccumulator()
    accumulator.add(
        PoseSample(1.0, 0.0, 0.0, math.radians(179)),
        PoseSample(1.0, 0.3, 0.4, math.radians(-179)),
    )
    summary = accumulator.summary()
    assert summary["position_error_m"]["mean"] == pytest.approx(0.5)
    assert summary["yaw_error_deg"]["mean"] == pytest.approx(2.0)


def test_summary_reports_median_and_p95() -> None:
    result = summarize([1, 2, 3, 4, 100])
    assert result["median"] == 3.0
    assert result["p95"] == pytest.approx(80.8)
