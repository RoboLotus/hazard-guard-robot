import pytest

from hazard_guard_safety_supervisor.policy import SafetyState
from hazard_guard_safety_supervisor.speed_limit import speed_limit_for_state


def test_clear_and_caution_remove_nav2_limit() -> None:
    assert speed_limit_for_state(SafetyState.CLEAR).speed_limit == 0.0
    assert speed_limit_for_state(SafetyState.CAUTION).speed_limit == 0.0


def test_slow_uses_configured_percentage() -> None:
    decision = speed_limit_for_state(SafetyState.SLOW, slow_percentage=42.0)
    assert decision.percentage
    assert decision.speed_limit == 42.0


@pytest.mark.parametrize("state", [SafetyState.STOP, SafetyState.SENSOR_FAULT])
def test_stop_states_never_publish_nav2_zero_limit(state: SafetyState) -> None:
    assert speed_limit_for_state(state).speed_limit == 1.0


@pytest.mark.parametrize(
    "slow, restrictive",
    [(0.0, 1.0), (101.0, 1.0), (50.0, 0.0), (30.0, 40.0)],
)
def test_rejects_invalid_percentages(slow: float, restrictive: float) -> None:
    with pytest.raises(ValueError):
        speed_limit_for_state(
            SafetyState.SLOW,
            slow_percentage=slow,
            restrictive_percentage=restrictive,
        )
