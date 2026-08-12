from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from statistics import median

from .geometry import normalize_angle


Pose2D = tuple[float, float, float]


class AlignmentDecision(str, Enum):
    ALIGNED = "aligned"
    ACCEPTED = "accepted"
    REALIGN = "realign"
    FAILED = "failed"
    FAILED_HARD = "failed_hard"


@dataclass(frozen=True)
class AlignmentThresholds:
    normal_position_m: float
    normal_yaw_rad: float
    acceptable_position_m: float
    acceptable_yaw_rad: float
    hard_position_m: float
    hard_yaw_rad: float

    def __post_init__(self) -> None:
        position_limits = (
            self.normal_position_m,
            self.acceptable_position_m,
            self.hard_position_m,
        )
        yaw_limits = (
            self.normal_yaw_rad,
            self.acceptable_yaw_rad,
            self.hard_yaw_rad,
        )
        if min(*position_limits, *yaw_limits) < 0.0:
            raise ValueError("alignment thresholds must be non-negative")
        if position_limits != tuple(sorted(position_limits)):
            raise ValueError("position thresholds must be ordered")
        if yaw_limits != tuple(sorted(yaw_limits)):
            raise ValueError("yaw thresholds must be ordered")


def decide_alignment(
    position_error_m: float,
    yaw_error_rad: float,
    thresholds: AlignmentThresholds,
    *,
    attempt: int,
    retries: int,
) -> AlignmentDecision:
    within_normal_position = position_error_m <= thresholds.normal_position_m
    within_normal_yaw = yaw_error_rad <= thresholds.normal_yaw_rad
    if within_normal_position and within_normal_yaw:
        return AlignmentDecision.ALIGNED
    within_acceptable_position = (
        position_error_m <= thresholds.acceptable_position_m
    )
    within_acceptable_yaw = yaw_error_rad <= thresholds.acceptable_yaw_rad
    if within_acceptable_position and within_acceptable_yaw:
        return AlignmentDecision.ACCEPTED
    if attempt < retries:
        return AlignmentDecision.REALIGN
    outside_hard_position = position_error_m > thresholds.hard_position_m
    outside_hard_yaw = yaw_error_rad > thresholds.hard_yaw_rad
    if outside_hard_position or outside_hard_yaw:
        return AlignmentDecision.FAILED_HARD
    return AlignmentDecision.FAILED


def median_pose(samples: list[Pose2D]) -> Pose2D:
    if not samples:
        raise ValueError("at least one pose sample is required")
    yaw = min(
        (sample[2] for sample in samples),
        key=lambda candidate: sum(
            abs(normalize_angle(candidate - sample[2])) for sample in samples
        ),
    )
    return (
        float(median(sample[0] for sample in samples)),
        float(median(sample[1] for sample in samples)),
        float(yaw),
    )


def sample_median_pose(
    read_pose: Callable[[], Pose2D | None],
    *,
    sample_count: int,
    min_valid_samples: int,
    interval_sec: float,
    sleep: Callable[[float], None] = time.sleep,
) -> Pose2D | None:
    count = max(1, int(sample_count))
    required = max(1, min(int(min_valid_samples), count))
    samples: list[Pose2D] = []
    for index in range(count):
        pose = read_pose()
        if pose is not None:
            samples.append(pose)
        if index + 1 < count and interval_sec > 0.0:
            sleep(interval_sec)
    if len(samples) < required:
        return None
    return median_pose(samples)
