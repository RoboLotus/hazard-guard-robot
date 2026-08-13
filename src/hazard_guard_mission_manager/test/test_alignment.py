import math

import pytest

from hazard_guard_mission_manager.alignment import (
    AlignmentDecision,
    AlignmentThresholds,
    decide_alignment,
    median_pose,
    sample_median_pose,
)


THRESHOLDS = AlignmentThresholds(
    normal_position_m=0.10,
    normal_yaw_rad=0.10,
    acceptable_position_m=0.15,
    acceptable_yaw_rad=0.17,
    hard_position_m=0.25,
    hard_yaw_rad=math.radians(15.0),
)


@pytest.mark.parametrize(
    ("position", "yaw", "attempt", "expected"),
    [
        (0.10, 0.10, 0, AlignmentDecision.ALIGNED),
        (0.15, 0.17, 0, AlignmentDecision.ACCEPTED),
        (0.16, 0.18, 0, AlignmentDecision.REALIGN),
        (0.16, 0.18, 1, AlignmentDecision.FAILED),
        (0.26, 0.18, 1, AlignmentDecision.FAILED_HARD),
    ],
)
def test_alignment_decisions_cover_boundaries_and_retry(
    position,
    yaw,
    attempt,
    expected,
):
    assert decide_alignment(
        position,
        yaw,
        THRESHOLDS,
        attempt=attempt,
        retries=1,
    ) == expected


def test_thresholds_must_be_ordered():
    with pytest.raises(ValueError, match="position thresholds"):
        AlignmentThresholds(0.2, 0.1, 0.1, 0.2, 0.3, 0.3)


def test_median_pose_rejects_outlier_and_handles_wrapped_yaw():
    result = median_pose(
        [
            (1.00, 2.00, math.radians(179.0)),
            (1.02, 2.01, math.radians(-179.0)),
            (8.00, -5.00, math.radians(178.0)),
        ]
    )

    assert result[:2] == (1.02, 2.00)
    assert abs(abs(math.degrees(result[2])) - 179.0) <= 1.0


def test_sampling_tolerates_one_tf_failure_and_uses_median():
    poses = iter(
        [
            (1.0, 2.0, 0.1),
            None,
            (1.1, 2.1, 0.1),
            (9.0, 9.0, 1.0),
            (1.2, 2.2, 0.1),
        ]
    )

    result = sample_median_pose(
        lambda: next(poses),
        sample_count=5,
        min_valid_samples=3,
        interval_sec=0.0,
    )

    assert result is not None
    assert result[:2] == pytest.approx((1.15, 2.15))
    assert result[2] == 0.1


def test_sampling_rejects_insufficient_tf_samples():
    poses = iter([None, (1.0, 2.0, 0.1), None])

    assert sample_median_pose(
        lambda: next(poses),
        sample_count=3,
        min_valid_samples=2,
        interval_sec=0.0,
    ) is None
