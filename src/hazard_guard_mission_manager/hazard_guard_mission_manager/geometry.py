from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the [-pi, pi] interval."""

    return math.atan2(math.sin(angle), math.cos(angle))


def pose_errors(
    actual: tuple[float, float, float],
    target: tuple[float, float, float],
) -> tuple[float, float]:
    """Return planar distance and absolute yaw errors."""

    position_error = math.hypot(target[0] - actual[0], target[1] - actual[1])
    yaw_error = abs(normalize_angle(target[2] - actual[2]))
    return position_error, yaw_error


def path_length(poses: list[object]) -> float:
    """Calculate the planar length of a nav_msgs/Path pose sequence."""

    distance = 0.0
    for previous, current in zip(poses, poses[1:]):
        previous_position = previous.pose.position
        current_position = current.pose.position
        distance += math.hypot(
            float(current_position.x) - float(previous_position.x),
            float(current_position.y) - float(previous_position.y),
        )
    return distance
