from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time
from typing import Any

from .coverage import CoverageAccumulator, MapGrid
from .environment import WorldAssets
from .metrics import (
    DetectionAccumulator,
    LocalizationAccumulator,
    PhaseTimer,
    PoseSample,
    SafetyAccumulator,
    SegmentAccumulator,
    TrajectoryAccumulator,
)


SAFE_ID = re.compile(r"[^A-Za-z0-9_.-]+")


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def safe_id(value: str) -> str:
    return SAFE_ID.sub("-", value.strip()).strip("-.")[:80] or "mission"


def default_storage_root() -> Path:
    configured = os.getenv("HAZARD_GUARD_BENCHMARK_DIR", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    workspace = os.getenv("HAZARD_GUARD_WORKSPACE", "").strip()
    if workspace:
        return (Path(workspace).expanduser() / "runtime" / "benchmarks").resolve()
    return (Path.home() / ".local/share/hazard-guard/benchmarks").resolve()


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class BenchmarkSession:
    def __init__(
        self,
        storage_root: Path,
        mission: dict[str, Any],
        assets: WorldAssets,
        map_grid: MapGrid,
        expected_heat_sources: set[str],
        metadata: dict[str, Any],
        inspection_radius_m: float,
        robot_clearance_m: float,
        minimum_step_m: float,
        maximum_step_m: float,
        coverage_sample_interval_sec: float = 1.0,
        near_miss_threshold_m: float = 0.35,
        scan_self_filter_min_m: float = 0.15,
    ) -> None:
        started = utc_now()
        mission_id = safe_id(str(mission.get("mission_id") or "mission"))
        base_id = f"{started.strftime('%Y%m%dT%H%M%SZ')}-{mission_id}"
        date_root = storage_root / started.strftime("%Y-%m-%d")
        report_id = base_id
        suffix = 1
        while True:
            self.directory = date_root / report_id
            try:
                self.directory.mkdir(parents=True, exist_ok=False)
                break
            except FileExistsError:
                suffix += 1
                report_id = f"{base_id}-{suffix}"
        self.report_id = report_id
        self.assets = assets
        self.map_grid = map_grid
        self.expected_heat_sources = set(expected_heat_sources)
        self.detected_heat_sources: set[str] = set()
        self.inspection_radius_m = inspection_radius_m
        self.robot_clearance_m = robot_clearance_m
        self.mission = dict(mission)
        self.metadata = dict(metadata)
        self.metadata.update(
            {
                "schema_version": 1,
                "id": report_id,
                "started_at": started.isoformat(),
                "world_id": assets.world_id,
                "world_name": assets.world_name,
                "mission_id": mission.get("mission_id"),
                "mission_name": mission.get("name"),
                "inspection_radius_m": inspection_radius_m,
                "robot_clearance_m": robot_clearance_m,
            }
        )
        self.trajectory = TrajectoryAccumulator(
            minimum_step_m=minimum_step_m,
            maximum_step_m=maximum_step_m,
        )
        self.coverage = CoverageAccumulator(
            map_grid,
            start_x=assets.spawn["x"],
            start_y=assets.spawn["y"],
            inspection_radius_m=inspection_radius_m,
            clearance_m=robot_clearance_m,
            sample_interval_sec=coverage_sample_interval_sec,
        )
        self.localization = LocalizationAccumulator()
        self.segments = SegmentAccumulator()
        self.safety = SafetyAccumulator(
            near_miss_threshold_m=near_miss_threshold_m,
            self_filter_min_m=scan_self_filter_min_m,
        )
        self.detections = DetectionAccumulator(set(expected_heat_sources))
        self.phases = PhaseTimer()
        self.collision_count = 0
        self.recovery_count = 0
        self.collision_available = False
        self.recovery_available = False
        self._collision_active = False
        self.global_plan_update_count = 0
        self._start_wall = time.monotonic()
        self._start_sim: float | None = None
        self._last_sim: float | None = None
        _atomic_json(self.directory / "metadata.json", self.metadata)

    def observe_mission(
        self,
        mission: dict[str, Any],
        *,
        timestamp_sec: float | None = None,
        pose: PoseSample | None = None,
    ) -> None:
        self.mission = dict(mission)
        self.segments.observe(
            mission,
            timestamp_sec=timestamp_sec,
            pose=pose,
            distance_m=self.trajectory.distance_m,
        )

    def add_ground_truth(self, sample: PoseSample) -> bool:
        if self._start_sim is None:
            self._start_sim = sample.timestamp_sec
        self._last_sim = sample.timestamp_sec
        self.detections.set_start(sample.timestamp_sec)
        self.phases.observe(
            str(self.mission.get("status") or "unknown"),
            sample.timestamp_sec,
        )
        accepted = self.trajectory.add(sample)
        self.segments.seed_origin(sample, self.trajectory.distance_m)
        self.coverage.add(sample)
        return accepted

    def add_localization(self, ground_truth: PoseSample, estimate: PoseSample) -> None:
        self.localization.add(ground_truth, estimate)

    def add_detection(
        self,
        detection_id: str,
        *,
        timestamp_sec: float | None = None,
        robot_pose: PoseSample | None = None,
        source_x: float | None = None,
        source_y: float | None = None,
        temperature_c: float | None = None,
        confidence: float | None = None,
    ) -> None:
        if detection_id:
            self.detected_heat_sources.add(detection_id)
            self.detections.add(
                detection_id,
                timestamp_sec=timestamp_sec,
                robot_pose=robot_pose,
                source_x=source_x,
                source_y=source_y,
                temperature_c=temperature_c,
                confidence=confidence,
            )

    def observe_scan(
        self,
        ranges: list[float],
        *,
        range_min: float,
        range_max: float,
    ) -> None:
        self.safety.add_scan(
            ranges,
            range_min=range_min,
            range_max=range_max,
        )

    def observe_global_plan(self) -> None:
        self.global_plan_update_count += 1

    def observe_collision(self, active: bool) -> None:
        self.collision_available = True
        if active and not self._collision_active:
            self.collision_count += 1
        self._collision_active = active

    def observe_recovery(self) -> None:
        self.recovery_available = True
        self.recovery_count += 1

    def _waypoint_summary(self) -> dict[str, Any]:
        waypoints = self.mission.get("waypoints")
        if not isinstance(waypoints, list):
            waypoints = []
        completed = [
            item
            for item in waypoints
            if isinstance(item, dict) and item.get("status") == "completed"
        ]
        failed = [
            str(item.get("id") or item.get("name") or "unknown")
            for item in waypoints
            if isinstance(item, dict) and item.get("status") == "failed"
        ]
        total = int(self.mission.get("total_waypoints") or len(waypoints))
        completed_count = max(
            len(completed), int(self.mission.get("completed_waypoints") or 0)
        )
        return {
            "total": total,
            "completed": completed_count,
            "completion_percent": round(
                completed_count / total * 100.0 if total else 0.0, 3
            ),
            "failed_ids": failed,
        }

    def finalize(self, result_status: str) -> dict[str, Any]:
        finished = utc_now()
        wall_duration = max(0.0, time.monotonic() - self._start_wall)
        sim_duration = (
            max(0.0, self._last_sim - self._start_sim)
            if self._last_sim is not None and self._start_sim is not None
            else 0.0
        )
        phase_durations = (
            self.phases.finalize(self._last_sim)
            if self._last_sim is not None
            else {}
        )
        self.segments.finalize(
            self._last_sim,
            self.trajectory.samples[-1] if self.trajectory.samples else None,
            self.trajectory.distance_m,
        )
        coverage, coverage_detail = self.coverage.finalize(
            self._last_sim if self._last_sim is not None else 0.0
        )
        planned_distance = self.mission.get("total_distance_m")
        planned_distance_m = (
            float(planned_distance)
            if isinstance(planned_distance, (int, float))
            else None
        )
        actual_distance_m = self.trajectory.distance_m
        path_efficiency = (
            planned_distance_m / actual_distance_m * 100.0
            if (
                result_status == "completed"
                and planned_distance_m is not None
                and actual_distance_m > 0.0
            )
            else None
        )
        thermal_summary = self.detections.summary()
        safety_summary = self.safety.summary()
        summary: dict[str, Any] = {
            "schema_version": 1,
            "id": self.report_id,
            "world_id": self.assets.world_id,
            "mission_id": self.mission.get("mission_id"),
            "mission_name": self.mission.get("name"),
            "status": result_status,
            "started_at": self.metadata["started_at"],
            "finished_at": finished.isoformat(),
            "time": {
                "simulation_sec": round(sim_duration, 3),
                "wall_sec": round(wall_duration, 3),
                "real_time_factor": round(sim_duration / wall_duration, 4)
                if wall_duration > 0.0
                else None,
                "phases_sec": phase_durations,
            },
            "trajectory": {
                "planned_distance_m": planned_distance_m,
                "actual_distance_m": round(actual_distance_m, 3),
                "path_efficiency_percent": round(path_efficiency, 3)
                if path_efficiency is not None
                else None,
                "sample_count": len(self.trajectory.samples),
                "dropped_jump_count": self.trajectory.dropped_jump_count,
            },
            "coverage": {**coverage.as_dict(), **coverage_detail},
            "waypoints": self._waypoint_summary(),
            "segments": self.segments.summary(),
            "thermal": thermal_summary,
            "safety": {
                "collision_count": self.collision_count
                if self.collision_available
                else None,
                "recovery_count": self.recovery_count
                if self.recovery_available
                else None,
                "global_plan_update_count": self.global_plan_update_count,
                **safety_summary,
            },
            "localization": self.localization.summary(),
            "reproducibility": self.metadata.get("reproducibility", {}),
        }
        _atomic_json(self.directory / "summary.json", summary)
        self._write_trajectory()
        self._write_coverage_timeseries()
        self._write_segments()
        self._write_detections()
        self._write_metrics_csv(summary)
        self._write_markdown(summary)
        return summary

    def _write_trajectory(self) -> None:
        with (self.directory / "trajectory.csv").open(
            "w", encoding="utf-8-sig", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["simulation_sec", "x", "y", "yaw_rad"])
            for sample in self.trajectory.samples:
                writer.writerow(
                    [
                        round(sample.timestamp_sec, 6),
                        round(sample.x, 6),
                        round(sample.y, 6),
                        round(sample.yaw, 6),
                    ]
                )

    def _write_coverage_timeseries(self) -> None:
        with (self.directory / "coverage_timeseries.csv").open(
            "w", encoding="utf-8-sig", newline=""
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=["elapsed_sec", "observed_area_m2", "coverage_percent"],
            )
            writer.writeheader()
            writer.writerows(sample.as_dict() for sample in self.coverage.samples)

    def _write_segments(self) -> None:
        fieldnames = [
            "cycle",
            "index",
            "waypoint_id",
            "waypoint_name",
            "status",
            "travel_time_sec",
            "dwell_time_sec",
            "total_time_sec",
            "direct_distance_m",
            "actual_distance_m",
            "path_efficiency_percent",
            "arrival_position_error_m",
            "arrival_yaw_error_deg",
        ]
        with (self.directory / "segments.csv").open(
            "w", encoding="utf-8-sig", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.segments.records)

    def _write_detections(self) -> None:
        fieldnames = [
            "detection_id",
            "expected",
            "elapsed_sec",
            "distance_m",
            "temperature_c",
            "confidence",
        ]
        with (self.directory / "detections.csv").open(
            "w", encoding="utf-8-sig", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.detections.events)

    def _write_metrics_csv(self, summary: dict[str, Any]) -> None:
        rows = [
            ["metric", "value", "unit"],
            ["simulation_time", summary["time"]["simulation_sec"], "sec"],
            ["wall_time", summary["time"]["wall_sec"], "sec"],
            ["real_time_factor", summary["time"]["real_time_factor"], "ratio"],
            ["planned_distance", summary["trajectory"]["planned_distance_m"], "m"],
            ["actual_distance", summary["trajectory"]["actual_distance_m"], "m"],
            ["path_efficiency", summary["trajectory"]["path_efficiency_percent"], "%"],
            ["patrolable_area", summary["coverage"]["patrolable_area_m2"], "m2"],
            ["observed_area", summary["coverage"]["observed_area_m2"], "m2"],
            ["space_coverage", summary["coverage"]["coverage_percent"], "%"],
            ["coverage_rate", summary["coverage"]["coverage_rate_m2_per_min"], "m2/min"],
            ["time_to_90_percent", summary["coverage"]["time_to_90_percent_sec"], "sec"],
            ["coverage_revisit", summary["coverage"]["revisit_percent"], "%"],
            ["waypoint_completion", summary["waypoints"]["completion_percent"], "%"],
            ["waypoint_arrival_position_error_p95", summary["segments"]["arrival_position_error_m"]["p95"], "m"],
            ["waypoint_arrival_yaw_error_p95", summary["segments"]["arrival_yaw_error_deg"]["p95"], "deg"],
            ["thermal_coverage", summary["thermal"]["coverage_percent"], "%"],
            ["thermal_precision", summary["thermal"]["precision"], "ratio"],
            ["thermal_first_detection", summary["thermal"]["first_expected_detection_sec"], "sec"],
            ["collision_count", summary["safety"]["collision_count"], "count"],
            ["recovery_count", summary["safety"]["recovery_count"], "count"],
            ["near_miss_count", summary["safety"]["near_miss_count"], "count"],
            ["minimum_clearance", summary["safety"]["minimum_clearance_m"], "m"],
            ["global_plan_updates", summary["safety"]["global_plan_update_count"], "count"],
        ]
        with (self.directory / "metrics.csv").open(
            "w", encoding="utf-8-sig", newline=""
        ) as stream:
            csv.writer(stream).writerows(rows)

    def _write_markdown(self, summary: dict[str, Any]) -> None:
        path_efficiency = summary["trajectory"]["path_efficiency_percent"]
        path_efficiency_text = (
            f"{path_efficiency}%" if path_efficiency is not None else "N/A"
        )
        lines = [
            f"# {summary['mission_name'] or summary['id']}",
            "",
            f"- 월드: `{summary['world_id']}`",
            f"- 결과: `{summary['status']}`",
            f"- 시뮬레이션 시간: {summary['time']['simulation_sec']:.1f}초",
            f"- 현실 경과 시간: {summary['time']['wall_sec']:.1f}초",
            f"- Real Time Factor: {summary['time']['real_time_factor']}",
            "",
            "## 순찰 결과",
            "",
            f"- 실제 이동 거리: {summary['trajectory']['actual_distance_m']:.2f}m",
            f"- 계획 이동 거리: {summary['trajectory']['planned_distance_m']}m",
            f"- 경로 효율: {path_efficiency_text}",
            f"- 점검점 완료율: {summary['waypoints']['completion_percent']}%",
            f"- 열원 관측률: {summary['thermal']['coverage_percent']}%",
            f"- 공간 커버리지: {summary['coverage']['coverage_percent']}%",
            f"- 관측 면적: {summary['coverage']['observed_area_m2']}m² / "
            f"{summary['coverage']['patrolable_area_m2']}m²",
            f"- 커버리지 증가 속도: {summary['coverage']['coverage_rate_m2_per_min']}m²/min",
            f"- 90% 관측 도달 시간: {summary['coverage']['time_to_90_percent_sec']}초",
            f"- 중복 방문 비율: {summary['coverage']['revisit_percent']}%",
            "",
            "## 안전 및 품질",
            "",
            f"- 충돌 횟수: {summary['safety']['collision_count']}",
            f"- Nav2 복구 횟수: {summary['safety']['recovery_count']}",
            f"- 근접 위험 횟수: {summary['safety']['near_miss_count']}",
            f"- 최소 장애물 거리: {summary['safety']['minimum_clearance_m']}m",
            f"- 전역 경로 갱신 횟수: {summary['safety']['global_plan_update_count']}",
            f"- 비정상 좌표 점프: {summary['trajectory']['dropped_jump_count']}회",
            f"- 누락 열원: {', '.join(summary['thermal']['missing_ids']) or '없음'}",
        ]
        (self.directory / "report.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
