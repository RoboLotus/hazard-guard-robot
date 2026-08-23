"""Pure safety-gate decisions for robot velocity commands."""

from .policy import SafetyState


def motion_is_allowed(
    state: SafetyState,
    *,
    state_fresh: bool,
    command_fresh: bool = True,
) -> bool:
    """Allow motion only with a fresh, non-stop safety state."""

    return state_fresh and command_fresh and state in {
        SafetyState.CLEAR,
        SafetyState.CAUTION,
        SafetyState.SLOW,
    }
