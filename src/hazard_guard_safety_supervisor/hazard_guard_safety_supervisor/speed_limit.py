"""Pure mapping from person-safety state to Nav2 speed limits."""

from dataclasses import dataclass

from .policy import SafetyState


@dataclass(frozen=True)
class SpeedLimitDecision:
    """A wire-independent representation of ``nav2_msgs/SpeedLimit``."""

    percentage: bool
    speed_limit: float


def speed_limit_for_state(
    state: SafetyState,
    *,
    slow_percentage: float = 45.0,
    restrictive_percentage: float = 1.0,
) -> SpeedLimitDecision:
    """Return the Nav2 limit for a safety state.

    Nav2 interprets a zero speed limit as *remove the limit*, not stop.  STOP
    and SENSOR_FAULT therefore use a tiny defensive limit here while the
    downstream ``cmd_vel`` safety gate performs the authoritative zeroing.
    """

    if not 0.0 < slow_percentage <= 100.0:
        raise ValueError("slow_percentage must be in (0, 100]")
    if not 0.0 < restrictive_percentage <= slow_percentage:
        raise ValueError(
            "restrictive_percentage must be in (0, slow_percentage]"
        )

    if state == SafetyState.SLOW:
        value = slow_percentage
    elif state in (SafetyState.STOP, SafetyState.SENSOR_FAULT):
        value = restrictive_percentage
    else:
        value = 0.0
    return SpeedLimitDecision(percentage=True, speed_limit=value)
