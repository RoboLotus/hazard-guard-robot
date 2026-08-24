import json
from pathlib import Path

from hazard_guard_patrol_benchmark.aggregate import aggregate_summaries


def _summary(path: Path, report_id: str, duration: float, status: str) -> Path:
    path.write_text(
        json.dumps(
            {
                "id": report_id,
                "status": status,
                "time": {"simulation_sec": duration},
                "trajectory": {
                    "actual_distance_m": 10.0,
                    "path_efficiency_percent": 90.0,
                },
                "coverage": {"coverage_percent": 50.0},
                "waypoints": {"completion_percent": 100.0},
                "thermal": {"coverage_percent": 100.0},
            }
        ),
        encoding="utf-8",
    )
    return path


def test_aggregate_summaries_counts_success_and_statistics(tmp_path: Path) -> None:
    paths = [
        _summary(tmp_path / "one.json", "one", 10.0, "completed"),
        _summary(tmp_path / "two.json", "two", 20.0, "failed"),
    ]
    summary = aggregate_summaries(paths)
    assert summary["run_count"] == 2
    assert summary["success_count"] == 1
    assert summary["metrics"]["simulation_sec"]["median"] == 15.0
