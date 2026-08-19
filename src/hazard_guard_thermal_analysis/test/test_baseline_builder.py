import json

import pytest

from hazard_guard_thermal_analysis.baseline import load_baselines
from hazard_guard_thermal_analysis.baseline_builder import (
    BaselineCollector,
    prepare_collector_after_topology_change,
)


def completed_visit(
    temperature_c: float,
    *,
    equipment_id: str = "motor",
    environment_temperature_c: float = 20.0,
    environment_points: int = 40,
    trend: bool = False,
    critical: bool = False,
) -> dict:
    return {
        "recorded_at_unix_sec": 1000.0 + temperature_c,
        "ambient": {
            "median_temperature_c": environment_temperature_c,
            "point_count": environment_points,
        },
        "equipment": [
            {
                "equipment_id": equipment_id,
                "p95_valid": True,
                "statistics": {
                    "p95_temperature_c": temperature_c,
                    "point_count": 50,
                },
                "voxels": [
                    {
                        "voxel_id": f"{equipment_id}:0:0:0",
                        "p95_temperature_c": temperature_c + 1.0,
                        "trend_analysis": {
                            "trend": trend,
                            "critical": critical,
                        },
                    }
                ],
            }
        ],
    }


def collector(tmp_path, minimum_valid_visits=10) -> BaselineCollector:
    return BaselineCollector(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor",),
        minimum_valid_visits=minimum_valid_visits,
        minimum_environment_points=40,
    )


def test_ten_visits_persist_and_create_validated_median_baseline(
    tmp_path,
) -> None:
    current = collector(tmp_path)
    for index in range(10):
        result = current.observe(completed_visit(30.0 + index * 0.1))
        assert result["accepted"] == ["motor"]

    assert current.ready is True
    assert current.counts() == {"motor": 10}

    restored = collector(tmp_path)
    assert restored.ready is True
    assert restored.counts() == {"motor": 10}
    assert restored.activate_if_ready() is True
    assert (tmp_path / "baselines.json").exists()

    baseline = load_baselines(tmp_path / "baselines.json")["motor"]
    assert baseline.equipment.temperature_c == pytest.approx(30.45)
    assert baseline.equipment.environment_delta_c == pytest.approx(10.45)
    assert baseline.equipment.sample_count == 10
    assert baseline.equipment.state == "validated"
    assert baseline.voxels["motor:0:0:0"].temperature_c == pytest.approx(
        31.45
    )


def test_latest_sample_time_and_approved_archive_are_recoverable(tmp_path) -> None:
    current = collector(tmp_path)
    for index in range(10):
        current.observe(completed_visit(30.0 + index * 0.1))

    assert current.latest_sample_times() == {"motor": 1030.9}
    assert current.activate_if_ready() is True

    archived = current.archive_approved()

    assert archived is not None
    assert archived.exists()
    assert not (tmp_path / "baselines.json").exists()
    assert load_baselines(archived)["motor"].equipment.sample_count == 10


def test_invalid_environment_quality_does_not_count(tmp_path) -> None:
    current = collector(tmp_path)
    result = current.observe(
        completed_visit(30.0, environment_points=39)
    )
    assert result["accepted"] == []
    assert result["reason"] == "invalid_reference_quality"
    assert current.counts() == {"motor": 0}
    assert not (tmp_path / "collection.json").exists()


def test_critical_pauses_without_deleting_prior_samples(tmp_path) -> None:
    current = collector(tmp_path, minimum_valid_visits=5)
    current.observe(completed_visit(30.0))
    current.observe(completed_visit(30.1))
    result = current.observe(completed_visit(30.2, critical=True))

    assert result["accepted"] == []
    assert result["paused"] == ["motor"]
    assert current.counts() == {"motor": 2}
    assert current.recovery_status()["motor"] == {
        "paused": True,
        "stable_streak": 0,
        "excluded_count": 1,
    }

    restored = collector(tmp_path, minimum_valid_visits=5)
    assert restored.counts() == {"motor": 2}
    assert restored.recovery_status()["motor"]["paused"] is True


def test_trend_excludes_only_the_recent_three_visit_window(
    tmp_path,
) -> None:
    current = collector(tmp_path)
    for index in range(5):
        current.observe(completed_visit(30.0 + index))

    result = current.observe(completed_visit(35.0, trend=True))

    assert result["paused"] == ["motor"]
    assert current.counts() == {"motor": 3}
    document = json.loads(
        (tmp_path / "collection.json").read_text(encoding="utf-8")
    )
    motor = document["equipment"]["motor"]
    assert len(motor["voxels"]["motor:0:0:0"]) == 3
    assert [item["reason"] for item in motor["excluded"]] == [
        "trend_recent_window",
        "trend_recent_window",
        "trend_detected",
    ]


def test_repeated_trend_does_not_remove_older_preserved_samples(
    tmp_path,
) -> None:
    current = collector(tmp_path)
    for index in range(5):
        current.observe(completed_visit(30.0 + index))
    current.observe(completed_visit(35.0, trend=True))
    assert current.counts() == {"motor": 3}

    current.observe(completed_visit(35.1))
    current.observe(completed_visit(35.2))
    result = current.observe(completed_visit(36.0, trend=True))

    assert result["paused"] == ["motor"]
    assert current.counts() == {"motor": 3}
    document = json.loads(
        (tmp_path / "collection.json").read_text(encoding="utf-8")
    )
    assert len(document["equipment"]["motor"]["samples"]) == 3


def test_three_stable_visits_resume_collection(tmp_path) -> None:
    current = collector(tmp_path, minimum_valid_visits=5)
    current.observe(completed_visit(30.0))
    current.observe(completed_visit(30.1))
    current.observe(completed_visit(31.0, critical=True))

    first = current.observe(completed_visit(30.2))
    second = current.observe(completed_visit(30.3))
    third = current.observe(completed_visit(30.4))

    assert first["accepted"] == []
    assert first["reason"] == "stability_recovery"
    assert second["accepted"] == []
    assert third["accepted"] == ["motor"]
    assert third["resumed"] == ["motor"]
    assert current.counts() == {"motor": 3}
    assert current.recovery_status()["motor"] == {
        "paused": False,
        "stable_streak": 0,
        "excluded_count": 3,
    }


def test_paused_complete_collection_is_not_ready_for_approval(
    tmp_path,
) -> None:
    current = collector(tmp_path, minimum_valid_visits=2)
    current.observe(completed_visit(30.0))
    current.observe(completed_visit(30.1))
    assert current.ready is True

    current.observe(completed_visit(31.0, critical=True))
    assert current.counts() == {"motor": 2}
    assert current.ready is False


def test_approval_is_rejected_until_every_equipment_is_ready(
    tmp_path,
) -> None:
    current = collector(tmp_path, minimum_valid_visits=3)
    current.observe(completed_visit(30.0))
    assert current.activate_if_ready() is False
    assert not (tmp_path / "baselines.json").exists()
    with pytest.raises(ValueError, match="not ready"):
        current.approve()


def test_every_configured_equipment_must_reach_the_minimum(tmp_path) -> None:
    current = BaselineCollector(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor", "pump"),
        minimum_valid_visits=2,
        minimum_environment_points=40,
    )
    current.observe(completed_visit(30.0, equipment_id="motor"))
    current.observe(completed_visit(30.1, equipment_id="motor"))
    assert current.counts() == {"motor": 2, "pump": 0}
    assert current.ready is False

    current.observe(completed_visit(40.0, equipment_id="pump"))
    current.observe(completed_visit(40.1, equipment_id="pump"))
    assert current.counts() == {"motor": 2, "pump": 2}
    assert current.ready is True


def test_reset_removes_persistent_collection(tmp_path) -> None:
    current = collector(tmp_path, minimum_valid_visits=3)
    current.observe(completed_visit(30.0))
    assert (tmp_path / "collection.json").exists()
    current.reset()
    assert current.counts() == {"motor": 0}
    assert not (tmp_path / "collection.json").exists()


def test_ready_equipment_activates_without_waiting_for_every_equipment(
    tmp_path,
) -> None:
    current = BaselineCollector(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor", "pump"),
        minimum_valid_visits=2,
        minimum_environment_points=40,
    )
    current.observe(completed_visit(30.0, equipment_id="motor"))
    current.observe(completed_visit(30.1, equipment_id="motor"))

    assert current.ready is False
    assert current.ready_equipment_ids == ("motor",)
    assert current.activate_ready_equipment() == ("motor",)
    first = json.loads(
        (tmp_path / "baselines.json").read_text(encoding="utf-8")
    )
    assert set(first["equipment"]) == {"motor"}

    current.observe(completed_visit(40.0, equipment_id="pump"))
    current.observe(completed_visit(40.1, equipment_id="pump"))
    assert current.activate_ready_equipment() == ("motor", "pump")
    completed = json.loads(
        (tmp_path / "baselines.json").read_text(encoding="utf-8")
    )
    assert set(completed["equipment"]) == {"motor", "pump"}


def test_roi_change_prunes_only_invalid_approved_baselines(tmp_path) -> None:
    current = BaselineCollector(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor", "pump"),
        minimum_valid_visits=2,
        minimum_environment_points=40,
    )
    for equipment_id, temperature in (("motor", 30.0), ("pump", 40.0)):
        current.observe(
            completed_visit(temperature, equipment_id=equipment_id)
        )
        current.observe(
            completed_visit(temperature + 0.1, equipment_id=equipment_id)
        )
    assert current.activate_ready_equipment() == ("motor", "pump")

    assert current.retain_approved_equipment(("pump",)) == ("pump",)
    retained = json.loads(
        (tmp_path / "baselines.json").read_text(encoding="utf-8")
    )
    assert set(retained["equipment"]) == {"pump"}

    current.reset()
    current.observe(completed_visit(31.0, equipment_id="motor"))
    current.observe(completed_visit(31.1, equipment_id="motor"))
    current.activate_ready_equipment()
    merged = json.loads(
        (tmp_path / "baselines.json").read_text(encoding="utf-8")
    )
    assert set(merged["equipment"]) == {"motor", "pump"}


def test_pruning_all_approved_baselines_removes_active_file(tmp_path) -> None:
    current = collector(tmp_path, minimum_valid_visits=2)
    current.observe(completed_visit(30.0))
    current.observe(completed_visit(30.1))
    assert current.activate_ready_equipment() == ("motor",)

    assert current.retain_approved_equipment(()) == ()
    assert not (tmp_path / "baselines.json").exists()


def test_topology_change_discards_corrupt_old_collection_before_load(
    tmp_path,
) -> None:
    current = BaselineCollector(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor", "pump"),
        minimum_valid_visits=2,
        minimum_environment_points=40,
    )
    for equipment_id, temperature in (("motor", 30.0), ("pump", 40.0)):
        current.observe(
            completed_visit(temperature, equipment_id=equipment_id)
        )
        current.observe(
            completed_visit(temperature + 0.1, equipment_id=equipment_id)
        )
    current.activate_ready_equipment()
    (tmp_path / "collection.json").write_text("{broken", encoding="utf-8")

    fresh = prepare_collector_after_topology_change(
        tmp_path / "collection.json",
        tmp_path / "baselines.json",
        ("motor", "pump"),
        ("pump",),
        minimum_valid_visits=2,
        minimum_environment_points=40,
    )

    assert fresh.counts() == {"motor": 0, "pump": 0}
    assert not (tmp_path / "collection.json").exists()
    retained = json.loads(
        (tmp_path / "baselines.json").read_text(encoding="utf-8")
    )
    assert set(retained["equipment"]) == {"pump"}
