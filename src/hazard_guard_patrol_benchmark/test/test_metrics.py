import math

import pytest

from hazard_guard_patrol_benchmark.metrics import (
    DetectionAccumulator,
    LocalizationAccumulator,
    PhaseTimer,
    PoseSample,
    SafetyAccumulator,
    SegmentAccumulator,
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


def test_safety_counts_near_miss_episodes_not_every_scan() -> None:
    safety = SafetyAccumulator(near_miss_threshold_m=0.35)
    safety.add_scan([1.0, 0.3], range_min=0.1, range_max=10.0)
    safety.add_scan([0.25], range_min=0.1, range_max=10.0)
    safety.add_scan([0.8], range_min=0.1, range_max=10.0)
    safety.add_scan([0.2], range_min=0.1, range_max=10.0)
    summary = safety.summary()
    assert summary["near_miss_count"] == 2
    assert summary["minimum_clearance_m"] == pytest.approx(0.2)
    assert summary["scan_message_count"] == 4
    assert summary["usable_scan_sample_count"] == 4


def test_safety_reports_scans_without_usable_ranges() -> None:
    safety = SafetyAccumulator(self_filter_min_m=0.15)
    safety.add_scan([0.12, float("inf")], range_min=0.15, range_max=12.0)

    summary = safety.summary()

    assert summary["scan_message_count"] == 1
    assert summary["usable_scan_sample_count"] == 0
    assert summary["finite_range_count"] == 1
    assert summary["filtered_range_count"] == 1
    assert summary["minimum_clearance_m"] is None


def test_detection_reports_precision_recall_latency_and_distance() -> None:
    detections = DetectionAccumulator({"hot-01", "hot-02"})
    detections.set_start(10.0)
    detections.add(
        "hot-01",
        timestamp_sec=12.0,
        robot_pose=PoseSample(12.0, 0.0, 0.0, 0.0),
        source_x=3.0,
        source_y=4.0,
    )
    detections.add("hot-01", timestamp_sec=13.0)
    detections.add("noise", timestamp_sec=14.0)
    summary = detections.summary()
    assert summary["recall"] == pytest.approx(0.5)
    assert summary["precision"] == pytest.approx(0.5)
    assert summary["duplicate_event_count"] == 1
    assert summary["first_expected_detection_sec"] == pytest.approx(2.0)
    assert summary["expected_detection_distance_m"]["mean"] == pytest.approx(5.0)


def test_segment_reports_arrival_error_and_path_efficiency() -> None:
    segments = SegmentAccumulator()
    mission = {
        "status": "executing",
        "current_cycle": 1,
        "current_index": 0,
        "waypoints": [
            {
                "id": "P01",
                "name": "pump",
                "x": 3.0,
                "y": 4.0,
                "yaw": 0.0,
                "status": "active",
            }
        ],
    }
    segments.observe(
        mission,
        timestamp_sec=10.0,
        pose=PoseSample(10.0, 0.0, 0.0, 0.0),
        distance_m=0.0,
    )
    mission["status"] = "dwelling"
    mission["waypoints"][0]["status"] = "dwelling"
    segments.observe(
        mission,
        timestamp_sec=15.0,
        pose=PoseSample(15.0, 3.1, 4.0, math.radians(2.0)),
        distance_m=6.0,
    )
    mission["status"] = "running"
    mission["waypoints"][0]["status"] = "completed"
    segments.observe(
        mission,
        timestamp_sec=17.0,
        pose=PoseSample(17.0, 3.1, 4.0, math.radians(2.0)),
        distance_m=6.0,
    )
    summary = segments.summary()
    assert summary["completed_count"] == 1
    assert summary["travel_time_sec"]["mean"] == pytest.approx(5.0)
    assert summary["arrival_position_error_m"]["mean"] == pytest.approx(0.1)
    assert segments.records[0]["path_efficiency_percent"] == pytest.approx(83.333)
