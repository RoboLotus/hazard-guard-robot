from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def summarize(values: Iterable[float]) -> dict[str, float | int | None]:
    numbers = [float(value) for value in values]
    if not numbers:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p95": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(numbers),
        "mean": round(sum(numbers) / len(numbers), 6),
        "median": round(float(percentile(numbers, 0.5)), 6),
        "p95": round(float(percentile(numbers, 0.95)), 6),
        "min": round(min(numbers), 6),
        "max": round(max(numbers), 6),
    }


def angular_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class PoseSample:
    timestamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class TrajectoryAccumulator:
    minimum_step_m: float = 0.01
    maximum_step_m: float = 2.0
    samples: list[PoseSample] = field(default_factory=list)
    distance_m: float = 0.0
    dropped_jump_count: int = 0

    def add(self, sample: PoseSample) -> bool:
        if self.samples and sample.timestamp_sec < self.samples[-1].timestamp_sec:
            return False
        if self.samples:
            previous = self.samples[-1]
            distance = math.hypot(sample.x - previous.x, sample.y - previous.y)
            if distance > self.maximum_step_m:
                self.dropped_jump_count += 1
                return False
            if distance < self.minimum_step_m:
                return False
            self.distance_m += distance
        self.samples.append(sample)
        return True


@dataclass
class PhaseTimer:
    durations: dict[str, float] = field(default_factory=dict)
    _phase: str | None = None
    _timestamp: float | None = None

    def observe(self, phase: str, timestamp_sec: float) -> None:
        if self._timestamp is not None and self._phase is not None:
            elapsed = max(0.0, timestamp_sec - self._timestamp)
            self.durations[self._phase] = self.durations.get(self._phase, 0.0) + elapsed
        self._phase = phase
        self._timestamp = timestamp_sec

    def finalize(self, timestamp_sec: float) -> dict[str, float]:
        self.observe(self._phase or "unknown", timestamp_sec)
        return {
            key: round(value, 3)
            for key, value in sorted(self.durations.items())
        }


@dataclass
class LocalizationAccumulator:
    position_errors_m: list[float] = field(default_factory=list)
    yaw_errors_rad: list[float] = field(default_factory=list)

    def add(self, ground_truth: PoseSample, estimate: PoseSample) -> None:
        self.position_errors_m.append(
            math.hypot(ground_truth.x - estimate.x, ground_truth.y - estimate.y)
        )
        self.yaw_errors_rad.append(angular_error(ground_truth.yaw, estimate.yaw))

    def summary(self) -> dict[str, dict[str, float | int | None]]:
        return {
            "position_error_m": summarize(self.position_errors_m),
            "yaw_error_deg": summarize(
                math.degrees(value) for value in self.yaw_errors_rad
            ),
        }


@dataclass
class SafetyAccumulator:
    near_miss_threshold_m: float = 0.35
    self_filter_min_m: float = 0.15
    clearance_samples_m: list[float] = field(default_factory=list)
    near_miss_count: int = 0
    scan_message_count: int = 0
    finite_range_count: int = 0
    filtered_range_count: int = 0
    _near_miss_active: bool = False

    def add_scan(
        self,
        ranges: Iterable[float],
        *,
        range_min: float = 0.0,
        range_max: float = math.inf,
    ) -> None:
        self.scan_message_count += 1
        finite = [float(value) for value in ranges if math.isfinite(float(value))]
        self.finite_range_count += len(finite)
        valid = [
            value
            for value in finite
            if value >= range_min and value <= range_max
        ]
        self.filtered_range_count += len(finite) - len(valid)
        if not valid:
            return
        clearance = min(valid)
        self.clearance_samples_m.append(clearance)
        active = clearance < self.near_miss_threshold_m
        if active and not self._near_miss_active:
            self.near_miss_count += 1
        self._near_miss_active = active

    def summary(self) -> dict[str, Any]:
        values = self.clearance_samples_m
        clearance = summarize(values)
        clearance["p05"] = (
            round(float(percentile(values, 0.05)), 6) if values else None
        )
        return {
            "minimum_clearance_m": min(values) if values else None,
            "clearance_m": clearance,
            "near_miss_threshold_m": self.near_miss_threshold_m,
            "self_filter_min_m": self.self_filter_min_m,
            "near_miss_count": self.near_miss_count if values else None,
            "scan_message_count": self.scan_message_count,
            "usable_scan_sample_count": len(values),
            "finite_range_count": self.finite_range_count,
            "filtered_range_count": self.filtered_range_count,
        }


@dataclass
class DetectionAccumulator:
    expected_ids: set[str]
    events: list[dict[str, Any]] = field(default_factory=list)
    _start_timestamp: float | None = None

    def set_start(self, timestamp_sec: float) -> None:
        if self._start_timestamp is None:
            self._start_timestamp = timestamp_sec

    def add(
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
        if not detection_id:
            return
        elapsed = None
        if timestamp_sec is not None and self._start_timestamp is not None:
            elapsed = max(0.0, timestamp_sec - self._start_timestamp)
        distance = None
        if (
            robot_pose is not None
            and source_x is not None
            and source_y is not None
        ):
            distance = math.hypot(robot_pose.x - source_x, robot_pose.y - source_y)
        self.events.append(
            {
                "detection_id": detection_id,
                "elapsed_sec": round(elapsed, 6) if elapsed is not None else None,
                "distance_m": round(distance, 6) if distance is not None else None,
                "temperature_c": temperature_c,
                "confidence": confidence,
                "expected": detection_id in self.expected_ids,
            }
        )

    def summary(self) -> dict[str, Any]:
        ids = [str(event["detection_id"]) for event in self.events]
        unique_ids = set(ids)
        detected_expected = unique_ids & self.expected_ids
        unexpected = unique_ids - self.expected_ids
        recall = (
            len(detected_expected) / len(self.expected_ids)
            if self.expected_ids
            else None
        )
        precision = (
            len(detected_expected) / len(unique_ids) if unique_ids else None
        )
        expected_events = [event for event in self.events if event["expected"]]
        elapsed_values = [
            float(event["elapsed_sec"])
            for event in expected_events
            if event["elapsed_sec"] is not None
        ]
        distance_values = [
            float(event["distance_m"])
            for event in expected_events
            if event["distance_m"] is not None
        ]
        return {
            "expected": len(self.expected_ids),
            "detected": len(detected_expected),
            "coverage_percent": round(recall * 100.0, 3)
            if recall is not None
            else 0.0,
            "recall": round(recall, 6) if recall is not None else None,
            "precision": round(precision, 6) if precision is not None else None,
            "missing_ids": sorted(self.expected_ids - detected_expected),
            "unexpected_ids": sorted(unexpected),
            "event_count": len(self.events),
            "duplicate_event_count": max(0, len(ids) - len(unique_ids)),
            "first_expected_detection_sec": min(elapsed_values)
            if elapsed_values
            else None,
            "expected_detection_distance_m": summarize(distance_values),
        }


@dataclass
class SegmentAccumulator:
    records: list[dict[str, Any]] = field(default_factory=list)
    _active: dict[str, Any] | None = None
    _origin_pose: PoseSample | None = None
    _origin_distance_m: float = 0.0
    _origin_timestamp_sec: float | None = None

    def seed_origin(self, pose: PoseSample, distance_m: float) -> None:
        if self._origin_pose is None and self._active is None:
            self._origin_pose = pose
            self._origin_distance_m = distance_m
            self._origin_timestamp_sec = pose.timestamp_sec

    @staticmethod
    def _waypoint(mission: dict[str, Any]) -> tuple[int, dict[str, Any]] | None:
        index = mission.get("current_index")
        waypoints = mission.get("waypoints")
        if not isinstance(index, int) or not isinstance(waypoints, list):
            return None
        if not 0 <= index < len(waypoints) or not isinstance(waypoints[index], dict):
            return None
        return index, waypoints[index]

    def observe(
        self,
        mission: dict[str, Any],
        *,
        timestamp_sec: float | None,
        pose: PoseSample | None,
        distance_m: float,
    ) -> None:
        selected = self._waypoint(mission)
        if selected is None or timestamp_sec is None:
            return
        index, waypoint = selected
        cycle = int(mission.get("current_cycle") or 1)
        key = (cycle, index)
        waypoint_status = str(waypoint.get("status") or "")
        mission_status = str(mission.get("status") or "")
        if self._active is not None and self._active["key"] != key:
            self._finish(timestamp_sec, pose, distance_m, "interrupted")
        if self._active is None:
            if mission_status != "executing" and waypoint_status != "active":
                return
            origin_pose = self._origin_pose or pose
            origin_distance = self._origin_distance_m if self._origin_pose else distance_m
            origin_timestamp = (
                self._origin_timestamp_sec
                if self._origin_timestamp_sec is not None
                else timestamp_sec
            )
            self._active = {
                "key": key,
                "cycle": cycle,
                "index": index,
                "waypoint_id": str(waypoint.get("id") or f"waypoint-{index + 1}"),
                "waypoint_name": str(waypoint.get("name") or ""),
                "target_x": float(waypoint.get("x") or 0.0),
                "target_y": float(waypoint.get("y") or 0.0),
                "target_yaw_rad": float(waypoint.get("yaw") or 0.0),
                "started_at_sec": origin_timestamp,
                "start_distance_m": origin_distance,
                "start_x": origin_pose.x if origin_pose is not None else None,
                "start_y": origin_pose.y if origin_pose is not None else None,
                "arrived_at_sec": None,
                "arrival_position_error_m": None,
                "arrival_yaw_error_deg": None,
            }
        if waypoint_status in {"dwelling", "completed"} and self._active[
            "arrived_at_sec"
        ] is None:
            self._active["arrived_at_sec"] = timestamp_sec
            if pose is not None:
                self._active["arrival_position_error_m"] = math.hypot(
                    pose.x - self._active["target_x"],
                    pose.y - self._active["target_y"],
                )
                self._active["arrival_yaw_error_deg"] = math.degrees(
                    angular_error(pose.yaw, self._active["target_yaw_rad"])
                )
        if waypoint_status == "completed":
            self._finish(timestamp_sec, pose, distance_m, "completed")
        elif waypoint_status == "failed":
            self._finish(timestamp_sec, pose, distance_m, "failed")

    def _finish(
        self,
        timestamp_sec: float,
        pose: PoseSample | None,
        distance_m: float,
        status: str,
    ) -> None:
        if self._active is None:
            return
        record = dict(self._active)
        record.pop("key", None)
        actual_distance = max(0.0, distance_m - float(record["start_distance_m"]))
        direct_distance = None
        if record["start_x"] is not None and record["start_y"] is not None:
            direct_distance = math.hypot(
                float(record["target_x"]) - float(record["start_x"]),
                float(record["target_y"]) - float(record["start_y"]),
            )
        arrived = record["arrived_at_sec"]
        record.update(
            {
                "status": status,
                "finished_at_sec": timestamp_sec,
                "travel_time_sec": round(
                    (arrived if arrived is not None else timestamp_sec)
                    - float(record["started_at_sec"]),
                    3,
                ),
                "dwell_time_sec": round(
                    timestamp_sec - float(arrived), 3
                )
                if arrived is not None
                else None,
                "total_time_sec": round(
                    timestamp_sec - float(record["started_at_sec"]), 3
                ),
                "actual_distance_m": round(actual_distance, 3),
                "direct_distance_m": round(direct_distance, 3)
                if direct_distance is not None
                else None,
                "path_efficiency_percent": round(
                    direct_distance / actual_distance * 100.0, 3
                )
                if direct_distance is not None and actual_distance > 0.0
                else None,
                "arrival_position_error_m": round(
                    float(record["arrival_position_error_m"]), 6
                )
                if record["arrival_position_error_m"] is not None
                else None,
                "arrival_yaw_error_deg": round(
                    float(record["arrival_yaw_error_deg"]), 6
                )
                if record["arrival_yaw_error_deg"] is not None
                else None,
            }
        )
        self.records.append(record)
        self._active = None
        if pose is not None:
            self._origin_pose = pose
            self._origin_distance_m = distance_m
            self._origin_timestamp_sec = timestamp_sec

    def finalize(
        self,
        timestamp_sec: float | None,
        pose: PoseSample | None,
        distance_m: float,
    ) -> None:
        if self._active is not None and timestamp_sec is not None:
            self._finish(timestamp_sec, pose, distance_m, "interrupted")

    def summary(self) -> dict[str, Any]:
        completed = [record for record in self.records if record["status"] == "completed"]
        return {
            "count": len(self.records),
            "completed_count": len(completed),
            "travel_time_sec": summarize(
                float(record["travel_time_sec"]) for record in completed
            ),
            "actual_distance_m": summarize(
                float(record["actual_distance_m"]) for record in completed
            ),
            "arrival_position_error_m": summarize(
                float(record["arrival_position_error_m"])
                for record in completed
                if record["arrival_position_error_m"] is not None
            ),
            "arrival_yaw_error_deg": summarize(
                float(record["arrival_yaw_error_deg"])
                for record in completed
                if record["arrival_yaw_error_deg"] is not None
            ),
        }
