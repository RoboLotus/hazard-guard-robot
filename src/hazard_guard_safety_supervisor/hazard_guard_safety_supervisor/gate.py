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


def command_path_is_allowed(
    state: SafetyState,
    *,
    state_fresh: bool,
    command_fresh: bool = True,
    require_safety_state: bool = True,
) -> bool:
    """Apply the command watchdog even when person supervision is disabled."""

    if not command_fresh:
        return False
    if not require_safety_state:
        return True
    return motion_is_allowed(
        state,
        state_fresh=state_fresh,
        command_fresh=True,
    )
