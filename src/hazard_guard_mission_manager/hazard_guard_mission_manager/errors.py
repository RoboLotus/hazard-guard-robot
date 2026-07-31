class MissionCanceled(RuntimeError):
    """Raised internally when the active patrol is canceled."""


class MissionFailure(RuntimeError):
    """Raised internally when a patrol step cannot be completed safely."""
