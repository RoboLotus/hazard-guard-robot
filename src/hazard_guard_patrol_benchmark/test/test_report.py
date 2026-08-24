import json
from pathlib import Path

from hazard_guard_patrol_benchmark.coverage import MapGrid
from hazard_guard_patrol_benchmark.environment import WorldAssets
from hazard_guard_patrol_benchmark.metrics import PoseSample
from hazard_guard_patrol_benchmark.report import BenchmarkSession


def test_session_writes_portable_reports(tmp_path: Path) -> None:
    repository = tmp_path / "Simulation_env"
    repository.mkdir()
    placeholder = repository / "asset"
    placeholder.write_text("asset", encoding="utf-8")
    assets = WorldAssets(
        repository_root=repository,
        world_id="test",
        world_name="test",
        world_path=placeholder,
        map_path=placeholder,
        heat_sources_path=None,
        patrol_script_path=None,
        spawn={"x": 0.5, "y": 0.5, "z": 0.0, "yaw": 0.0},
        catalog_entry={},
    )
    grid = MapGrid(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        free_mask=bytearray([1] * 9),
    )
    mission = {
        "mission_id": "test-mission",
        "name": "test patrol",
        "status": "running",
        "total_waypoints": 1,
        "completed_waypoints": 0,
        "total_distance_m": 1.0,
        "waypoints": [{"id": "P01", "status": "pending"}],
    }
    session = BenchmarkSession(
        tmp_path / "results",
        mission,
        assets,
        grid,
        {"hot-01"},
        {"reproducibility": {"robot_commit": "abc"}},
        inspection_radius_m=0.49,
        robot_clearance_m=0.0,
        minimum_step_m=0.01,
        maximum_step_m=2.0,
    )
    session.add_ground_truth(PoseSample(10.0, 0.5, 0.5, 0.0))
    session.add_ground_truth(PoseSample(12.0, 1.5, 0.5, 0.0))
    session.add_detection("hot-01")
    mission["status"] = "completed"
    mission["completed_waypoints"] = 1
    mission["waypoints"][0]["status"] = "completed"
    session.observe_mission(mission)
    summary = session.finalize("completed")
    assert summary["time"]["simulation_sec"] == 2.0
    assert summary["trajectory"]["actual_distance_m"] == 1.0
    assert summary["waypoints"]["completion_percent"] == 100.0
    assert summary["thermal"]["coverage_percent"] == 100.0
    assert (session.directory / "metrics.csv").is_file()
    assert (session.directory / "trajectory.csv").is_file()
    assert (session.directory / "coverage_timeseries.csv").is_file()
    assert (session.directory / "segments.csv").is_file()
    assert (session.directory / "detections.csv").is_file()
    assert (session.directory / "report.md").is_file()
    written = json.loads(
        (session.directory / "summary.json").read_text(encoding="utf-8")
    )
    assert written["reproducibility"]["robot_commit"] == "abc"


def test_failed_session_does_not_report_misleading_path_efficiency(
    tmp_path: Path,
) -> None:
    repository = tmp_path / "Simulation_env"
    repository.mkdir()
    placeholder = repository / "asset"
    placeholder.write_text("asset", encoding="utf-8")
    assets = WorldAssets(
        repository_root=repository,
        world_id="test",
        world_name="test",
        world_path=placeholder,
        map_path=placeholder,
        heat_sources_path=None,
        patrol_script_path=None,
        spawn={"x": 0.5, "y": 0.5, "z": 0.0, "yaw": 0.0},
        catalog_entry={},
    )
    grid = MapGrid(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        free_mask=bytearray([1] * 9),
    )
    mission = {
        "mission_id": "failed-mission",
        "name": "failed patrol",
        "status": "running",
        "total_waypoints": 3,
        "completed_waypoints": 0,
        "total_distance_m": 20.0,
        "waypoints": [{"id": "P01", "status": "active"}],
    }
    session = BenchmarkSession(
        tmp_path / "results",
        mission,
        assets,
        grid,
        set(),
        {},
        inspection_radius_m=0.49,
        robot_clearance_m=0.0,
        minimum_step_m=0.01,
        maximum_step_m=2.0,
    )
    session.add_ground_truth(PoseSample(10.0, 0.5, 0.5, 0.0))
    session.add_ground_truth(PoseSample(11.0, 1.5, 0.5, 0.0))

    summary = session.finalize("failed")

    assert summary["trajectory"]["actual_distance_m"] == 1.0
    assert summary["trajectory"]["path_efficiency_percent"] is None
    report = (session.directory / "report.md").read_text(encoding="utf-8")
    assert "경로 효율: N/A" in report
