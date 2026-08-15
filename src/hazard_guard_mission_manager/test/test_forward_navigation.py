import math
from types import SimpleNamespace

from hazard_guard_mission_manager.node import HazardGuardMissionManager


class FakeNavigation:
    def __init__(self, pose):
        self.pose = pose

    def current_pose(self, _frame_id):
        return self.pose


def make_manager(current_pose):
    manager = object.__new__(HazardGuardMissionManager)
    manager._nav = FakeNavigation(current_pose)
    parameter_values = {
        "forward_approach_min_distance_m": 0.15,
        "yaw_tolerance_rad": 0.10,
        "alignment_timeout_sec": 45.0,
    }

    def get_parameter(name):
        return SimpleNamespace(value=parameter_values[name])

    manager.navigation_calls = []
    manager.state_updates = []
    manager.waypoint_updates = []

    def navigate(target, frame_id, goal, *, timeout):
        manager.navigation_calls.append((target, frame_id, goal, timeout))

    def update_state(**values):
        manager.state_updates.append(values)

    def update_waypoint(index, status, message, **details):
        manager.waypoint_updates.append((index, status, message, details))

    manager.get_parameter = get_parameter
    manager._navigate_with_safety_retry = navigate
    manager._update_state = update_state
    manager._update_waypoint = update_waypoint
    return manager


def test_waypoint_navigation_uses_forward_transit_then_inspection_heading():
    manager = make_manager((0.0, 0.0, math.pi))
    target = (2.0, 2.0, -math.pi / 2.0)

    manager._navigate_forward_to_pose(
        target,
        "map",
        "mission-goal",
        timeout=180.0,
        waypoint_index=1,
        waypoint_name="설비 B",
    )

    assert len(manager.navigation_calls) == 2
    approach = manager.navigation_calls[0][0]
    inspection = manager.navigation_calls[1][0]
    assert approach[:2] == target[:2]
    assert math.isclose(approach[2], math.pi / 4.0)
    assert inspection == target
    assert manager.navigation_calls[0][3] == 180.0
    assert manager.navigation_calls[1][3] == 45.0
    assert [item[1] for item in manager.waypoint_updates] == [
        "active",
        "aligning",
    ]


def test_short_waypoint_leg_uses_one_final_pose_goal():
    manager = make_manager((0.0, 0.0, 0.0))
    target = (0.05, 0.0, math.pi)

    manager._navigate_forward_to_pose(
        target,
        "map",
        "mission-goal",
        timeout=180.0,
    )

    assert [call[0] for call in manager.navigation_calls] == [target]
