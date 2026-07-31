import math
from types import SimpleNamespace

from hazard_guard_mission_manager.geometry import (
    normalize_angle,
    path_length,
    pose_errors,
)


def make_pose(x: float, y: float):
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
    )


def test_normalize_angle_wraps_to_pi_interval():
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-9)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-9)


def test_pose_errors_separates_position_and_heading():
    position, yaw = pose_errors((0.0, 0.0, 0.0), (3.0, 4.0, math.pi / 2))
    assert position == 5.0
    assert math.isclose(yaw, math.pi / 2)


def test_path_length_sums_segments():
    assert path_length([make_pose(0, 0), make_pose(3, 4), make_pose(6, 8)]) == 10
