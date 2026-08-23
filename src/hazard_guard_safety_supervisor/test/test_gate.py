import pytest

from hazard_guard_safety_supervisor.gate import motion_is_allowed
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
