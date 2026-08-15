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


def forward_approach_pose(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    *,
    minimum_distance_m: float,
) -> tuple[float, float, float]:
    """Point the transit goal toward its position, not its inspection yaw.

    Nav2 otherwise receives the inspection heading while it is still driving
    between waypoints and may decide that reversing is the shortest solution.
    Very short legs retain the final heading because there is no meaningful
    travel direction to align with.
    """

    delta_x = float(target[0]) - float(current[0])
    delta_y = float(target[1]) - float(current[1])
    if math.hypot(delta_x, delta_y) <= max(0.0, minimum_distance_m):
        return target
    return target[0], target[1], math.atan2(delta_y, delta_x)


def heading_change_required(
    approach: tuple[float, float, float],
    target: tuple[float, float, float],
    *,
    tolerance_rad: float,
) -> bool:
    """Return whether arrival needs a separate inspection-heading goal."""

    return abs(normalize_angle(target[2] - approach[2])) > max(
        0.0,
        tolerance_rad,
    )


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
