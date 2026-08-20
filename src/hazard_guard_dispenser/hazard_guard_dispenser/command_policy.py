"""Keep bench-only servo commands out of the production command path."""


def allow_legacy_drop(enabled: bool) -> bool:
    return bool(enabled)


def allow_maintenance_command(command: str, enabled: bool) -> bool:
    """Only ``home`` and ``angle:NN`` need the physical maintenance gate."""

    normalized = command.strip().lower()
    return bool(enabled) and (normalized == "home" or normalized.startswith("angle:"))


def physical_drop_block_reason(
    *,
    enabled: bool,
    hardware_available: bool,
    motion_stopped: bool = True,
    armed_count: int | None = None,
) -> str | None:
    """Return the fail-closed reason that prevents physical actuation."""

    if not enabled:
        return "physical_drop_disabled"
    if not hardware_available:
        return "hardware_unavailable"
    if not motion_stopped:
        return "robot_not_stably_stopped"
    if armed_count is not None and armed_count < 1:
        return "no_ble_confirmation_channel"
    return None
