class MissionCanceled(RuntimeError):
    """Raised internally when the active patrol is canceled."""


class MissionScheduleEnded(MissionCanceled):
    """Raised when a wall-clock patrol window reaches its end."""


class MissionFailure(RuntimeError):
    """Raised internally when a patrol step cannot be completed safely."""
