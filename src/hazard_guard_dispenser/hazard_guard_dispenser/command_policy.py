"""Keep bench-only servo commands out of the production command path."""


def allow_legacy_drop(enabled: bool) -> bool:
    return bool(enabled)


def allow_maintenance_command(command: str, enabled: bool) -> bool:
    """Only ``home`` and ``angle:NN`` need the physical maintenance gate."""

    normalized = command.strip().lower()
    return bool(enabled) and (normalized == "home" or normalized.startswith("angle:"))
