from hazard_guard_mock_robot.state import RobotState


def test_pause_resume_and_stop_are_deterministic():
    state = RobotState()

    assert state.apply_command("pause")[0] is True
    assert state.mode == "paused"
    assert state.snapshot(0)["speed_mps"] == 0.0

    assert state.apply_command("resume")[0] is True
    assert state.mode == "patrol"
    assert state.snapshot(0)["speed_mps"] == 0.32

    assert state.apply_command("stop")[0] is True
    assert state.mode == "stopped"
    assert state.apply_command("resume")[0] is False


def test_controller_toggle_never_disables_mock_flag():
    state = RobotState()

    assert state.apply_command("controller", True)[0] is True
    assert state.controller_enabled is True
    assert state.snapshot(1)["mock"] is True

    assert state.apply_command("controller_off")[0] is True
    assert state.controller_enabled is False


def test_unknown_command_is_rejected():
    state = RobotState()
    accepted, message = state.apply_command("deploy-warning-device")
    assert accepted is False
    assert "지원하지 않는" in message


def test_simulation_measurements_override_synthetic_motion():
    state = RobotState(measured_speed_mps=0.17, lidar_hz=9.8, lidar_status="normal")

    snapshot = state.snapshot(0)

    assert snapshot["speed_mps"] == 0.17
    assert snapshot["lidar_hz"] == 9.8
    assert snapshot["lidar_status"] == "normal"
