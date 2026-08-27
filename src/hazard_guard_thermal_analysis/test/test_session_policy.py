import json
from pathlib import Path

import pytest

from hazard_guard_thermal_analysis.session_policy import (
    normalize_optional_session_id,
    validate_equipment_map_session,
)


def test_matching_active_map_session_is_accepted() -> None:
    validate_equipment_map_session(
        active_session_id="map-a",
        equipment_session_id="map-a",
        empty_disable=False,
    )


def test_equipment_session_is_rejected_when_active_session_is_unknown() -> None:
    with pytest.raises(ValueError, match="active patrol map_session_id"):
        validate_equipment_map_session(
            active_session_id="",
            equipment_session_id="map-a",
            empty_disable=False,
        )


def test_sessionless_empty_equipment_can_still_disable_analysis() -> None:
    validate_equipment_map_session(
        active_session_id="",
        equipment_session_id="",
        empty_disable=True,
    )


@pytest.mark.parametrize("schema_version", [1, 0])
def test_legacy_nonempty_equipment_requires_session_aware_migration(
    schema_version: int,
) -> None:
    with pytest.raises(ValueError, match="migration required"):
        validate_equipment_map_session(
            active_session_id="map-a",
            equipment_session_id="",
            empty_disable=False,
            schema_version=schema_version,
        )


def test_legacy_empty_equipment_can_explicitly_disable_analysis() -> None:
    validate_equipment_map_session(
        active_session_id="map-a",
        equipment_session_id="",
        empty_disable=True,
        schema_version=1,
    )


def test_json_null_session_allows_empty_equipment_disable() -> None:
    document = json.loads(
        '{"schema_version": 2, "map_session_id": null, "equipment": []}'
    )

    validate_equipment_map_session(
        active_session_id="map-a",
        equipment_session_id=normalize_optional_session_id(
            document.get("map_session_id")
        ),
        empty_disable=document["equipment"] == [],
        schema_version=document["schema_version"],
    )


@pytest.mark.parametrize("value", [0, False, [], {}])
def test_non_string_session_identity_is_rejected(value: object) -> None:
    with pytest.raises(ValueError, match="string or null"):
        normalize_optional_session_id(value)


def test_analyzer_validates_session_before_legacy_schema_branch() -> None:
    source = (
        Path(__file__).parents[1]
        / "hazard_guard_thermal_analysis"
        / "analyzer_node.py"
    ).read_text(encoding="utf-8")
    callback = source.index("def _on_equipment_config")
    validation = source.index(
        "validate_equipment_map_session(", callback
    )
    frame_check = source.index("if schema_version >= 2", callback)
    rejection_log = source.index(
        'self.get_logger().error(f"Rejected equipment configuration:',
        callback,
    )

    assert callback < validation < frame_check < rejection_log
