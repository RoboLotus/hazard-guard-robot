"""Keep bench-only servo commands out of the production command path."""


def allow_legacy_drop(enabled: bool) -> bool:
    return bool(enabled)


def allow_maintenance_command(command: str, enabled: bool) -> bool:
    """Gate the manual commands an operator issues at the robot.

    ``reset_installed`` is checked by this gate in ``node.py`` but was missing
    from the allowed set, so it could never run whatever the parameter said. A
    cube that was picked up and physically reloaded stayed marked as deployed
    forever and was excluded from every later drop.

    This command clears deployment bookkeeping only; it moves nothing. The
    physical actuation gate is ``physical_drop_block_reason`` below and is
    unchanged, so an operator still cannot drive the servo through this path.
    """

    normalized = command.strip().lower()
    return bool(enabled) and (
        normalized == "home"
        or normalized.startswith("angle:")
        or normalized == "reset_installed"
        or normalized.startswith("reset_installed:")
    )


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
