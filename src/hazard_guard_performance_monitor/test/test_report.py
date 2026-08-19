from datetime import datetime, timezone
import json
from pathlib import Path

from hazard_guard_performance_monitor.report import (
    PerformanceSession,
    summarize_session,
)


def sample(elapsed: float, phase: str = "executing") -> dict:
    return {
        "timestamp": f"2026-08-18T00:00:0{int(elapsed)}+00:00",
        "elapsed_sec": elapsed,
        "phase": phase,
        "cpu": {"total_percent": 50.0 + elapsed, "cores": {"cpu0": 70.0}},
        "memory": {
            "used_percent": 60.0,
            "used_mb": 4096.0,
            "swap_used_mb": 0.0,
        },
        "jetson": {
            "gpu_percent": 40.0,
            "temperatures_c": {"cpu": 55.0, "gpu": 52.0},
            "power_mw": {"vdd_in": 8000.0},
        },
        "processes": [
            {
                "label": "rtabmap",
                "pid": 10,
                "cpu_core_percent": 125.0,
                "rss_mb": 900.0,
                "threads": 8,
            }
        ],
    }


def test_summary_contains_system_core_process_and_phase_statistics():
    metadata = {
        "id": "report-1",
        "name": "정기 순찰",
        "mission_id": "mission-1",
        "mission_name": "정기 순찰",
        "started_at": "2026-08-18T00:00:00+00:00",
        "platform": {"architecture": "aarch64"},
    }

    summary = summarize_session(
        metadata,
        [sample(1.0), sample(2.0, "dwelling")],
        "completed",
        "2026-08-18T00:00:03+00:00",
    )

    assert summary["system"]["cpu_percent"]["mean"] == 51.5
    assert summary["cores"]["cpu0"]["p95"] == 70.0
    assert summary["processes"][0]["label"] == "rtabmap"
    assert set(summary["phases"]) == {"dwelling", "executing"}


def test_summary_sums_processes_with_the_same_label_per_sample():
    metadata = {
        "id": "report-1",
        "name": "Patrol",
        "started_at": "2026-08-18T00:00:00+00:00",
    }
    first = sample(1.0)
    first["processes"].append(
        {
            "label": "rtabmap",
            "pid": 11,
            "cpu_core_percent": 25.0,
            "rss_mb": 100.0,
            "threads": 2,
        }
    )

    summary = summarize_session(
        metadata,
        [first],
        "completed",
        "2026-08-18T00:00:02+00:00",
    )

    process = summary["processes"][0]
    assert process["cpu_core_percent"]["mean"] == 150.0
    assert process["rss_mb"]["mean"] == 1000.0
    assert process["threads"]["mean"] == 10.0


def test_session_writes_raw_and_final_report_files(tmp_path: Path):
    session = PerformanceSession(
        tmp_path,
        {"mission_id": "mission-1", "name": "정기 순찰"},
        {"architecture": "aarch64"},
        datetime(2026, 8, 18, tzinfo=timezone.utc),
    )
    session.append(sample(1.0))

    assert (session.directory / "active.json").exists()
    summary = session.finalize("completed")

    assert summary["sample_count"] == 1
    assert not (session.directory / "active.json").exists()
    assert (session.directory / "samples.jsonl").exists()
    assert (session.directory / "process-summary.csv").exists()
    assert (session.directory / "report.md").exists()
    persisted = json.loads(
        (session.directory / "summary.json").read_text(encoding="utf-8")
    )
    assert persisted["status"] == "completed"


def test_session_uses_a_suffix_when_report_id_exists(tmp_path: Path):
    started_at = datetime(2026, 8, 18, 3, 0, tzinfo=timezone.utc)
    mission = {"mission_id": "mission-1", "name": "Morning patrol"}
    first = PerformanceSession(tmp_path, mission, {}, started_at=started_at)
    first.finalize("completed", finished_at=started_at)

    second = PerformanceSession(tmp_path, mission, {}, started_at=started_at)

    assert second.metadata["id"].endswith("-2")
    second.discard()
