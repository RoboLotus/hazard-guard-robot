"""Fail-closed map-session checks shared by thermal analysis inputs."""


def normalize_optional_session_id(value: object) -> str:
    """Normalize JSON null while rejecting ambiguous non-string identities."""

    if value is None:
        return ""
    if not isinstance(value, str):
        raise ValueError("map_session_id must be a string or null")
    return value.strip()


def validate_equipment_map_session(
    *,
    active_session_id: str,
    equipment_session_id: str,
    empty_disable: bool,
    schema_version: int = 2,
) -> None:
    """Reject map-bound equipment unless its active map identity is known."""

    active = normalize_optional_session_id(active_session_id)
    received = normalize_optional_session_id(equipment_session_id)
    if empty_disable and not received:
        return
    if int(schema_version) < 2:
        raise ValueError(
            "equipment configuration migration required: non-empty map-bound "
            "equipment must use schema_version >= 2 with map_session_id"
        )
    if not received:
        raise ValueError("map-bound equipment needs map_session_id")
    if not active:
        raise ValueError(
            "active patrol map_session_id is unavailable; "
            "map-bound equipment cannot be validated"
        )
    if received != active:
        raise ValueError(
            "equipment map_session_id does not match active patrol session: "
            f"{received!r} != {active!r}"
        )
