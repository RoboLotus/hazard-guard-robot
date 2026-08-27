from pathlib import Path

import pytest

from hazard_guard_safety_supervisor.gate import (
    command_path_is_allowed,
    motion_is_allowed,
)
from hazard_guard_safety_supervisor.policy import SafetyState


@pytest.mark.parametrize(
    "state",
    [SafetyState.CLEAR, SafetyState.CAUTION, SafetyState.SLOW],
)
def test_fresh_non_stop_state_allows_nav2_command(state: SafetyState) -> None:
    assert motion_is_allowed(state, state_fresh=True)


@pytest.mark.parametrize(
    "state",
    [SafetyState.STOP, SafetyState.SENSOR_FAULT],
)
def test_stop_and_fault_block_motion(state: SafetyState) -> None:
    assert not motion_is_allowed(state, state_fresh=True)


def test_stale_state_blocks_even_if_last_state_was_clear() -> None:
    assert not motion_is_allowed(SafetyState.CLEAR, state_fresh=False)


def test_stale_velocity_input_blocks_even_if_safety_is_clear() -> None:
    assert not motion_is_allowed(
        SafetyState.CLEAR,
        state_fresh=True,
        command_fresh=False,
    )


def test_command_watchdog_allows_fresh_nav2_without_person_supervision() -> None:
    assert command_path_is_allowed(
        SafetyState.SENSOR_FAULT,
        state_fresh=False,
        command_fresh=True,
        require_safety_state=False,
    )


def test_command_watchdog_stops_after_nav2_input_expires() -> None:
    assert not command_path_is_allowed(
        SafetyState.CLEAR,
        state_fresh=True,
        command_fresh=False,
        require_safety_state=False,
    )


def test_process_signals_publish_zero_before_destroying_gate() -> None:
    source = (
        Path(__file__).parents[1]
        / "hazard_guard_safety_supervisor"
        / "gate_node.py"
    ).read_text(encoding="utf-8")

    assert "SignalHandlerOptions.NO" in source
    assert "signal.SIGINT, signal.SIGTERM" in source
    handler = source.index("def request_shutdown")
    immediate_stop = source.index("node.publish_stop_burst()", handler)
    finally_block = source.index("finally:", immediate_stop)
    final_stop = source.index("node.publish_stop_burst()", finally_block)
    destroy = source.index("node.destroy_node()")
    shutdown = source.index("rclpy.shutdown()")
    assert handler < immediate_stop < finally_block < final_stop
    assert final_stop < destroy < shutdown
