import math
from types import SimpleNamespace

from hazard_guard_mission_manager.geometry import (
    forward_approach_pose,
    heading_change_required,
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
    poses = [make_pose(0, 0), make_pose(3, 4), make_pose(6, 8)]
    assert path_length(poses) == 10


def test_forward_approach_faces_the_next_waypoint_instead_of_inspection_yaw():
    approach = forward_approach_pose(
        (0.0, 0.0, math.pi),
        (2.0, 2.0, -math.pi / 2.0),
        minimum_distance_m=0.15,
    )

    assert approach[:2] == (2.0, 2.0)
    assert math.isclose(approach[2], math.pi / 4.0)
    assert heading_change_required(
        approach,
        (2.0, 2.0, -math.pi / 2.0),
        tolerance_rad=0.10,
    )


def test_short_leg_keeps_final_heading_without_a_redundant_transit_goal():
    target = (0.05, 0.0, math.pi)
    approach = forward_approach_pose(
        (0.0, 0.0, 0.0),
        target,
        minimum_distance_m=0.15,
    )

    assert approach == target
    assert not heading_change_required(
        approach,
        target,
        tolerance_rad=0.10,
    )


def test_heading_change_wraps_across_pi_boundary():
    assert not heading_change_required(
        (1.0, 1.0, math.radians(179.0)),
        (1.0, 1.0, math.radians(-179.0)),
        tolerance_rad=math.radians(3.0),
    )
