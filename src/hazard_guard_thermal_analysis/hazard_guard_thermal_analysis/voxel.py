from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from statistics import fmean, median
from typing import Iterable, Mapping, Sequence

from .projection import ThermalPoint


@dataclass(frozen=True)
class AxisAlignedRoi:
    roi_id: str
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    warning_temperature_c: float | None = None
    critical_temperature_c: float | None = None
    warning_delta_c: float | None = None

    def contains(self, x: float, y: float, z: float) -> bool:
        return all(
            low <= value <= high
            for value, low, high in zip(
                (x, y, z), self.minimum, self.maximum
            )
        )


@dataclass(frozen=True)
class AnalysisConfig:
    frame_id: str
    voxel_size_m: float
    min_points_per_voxel: int
    equipment_rois: tuple[AxisAlignedRoi, ...]
    environment_rois: tuple[AxisAlignedRoi, ...]
    min_confidence: float = 0.25
    schema_version: int = 1


def _vector3(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain three numbers")
    if len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result  # type: ignore[return-value]


def _parse_roi(value: object) -> AxisAlignedRoi:
    if not isinstance(value, Mapping):
        raise ValueError("ROI entries must be objects")
    minimum = _vector3(value.get("min"), "ROI min")
    maximum = _vector3(value.get("max"), "ROI max")
    if any(low >= high for low, high in zip(minimum, maximum)):
        raise ValueError("each ROI min value must be smaller than max")

    def optional_float(name: str) -> float | None:
        raw = value.get(name)
        return None if raw is None else float(raw)

    return AxisAlignedRoi(
        roi_id=str(value.get("id", "")).strip(),
        minimum=minimum,
        maximum=maximum,
        warning_temperature_c=optional_float("warning_temperature_c"),
        critical_temperature_c=optional_float("critical_temperature_c"),
        warning_delta_c=optional_float("warning_delta_c"),
    )


def load_config(path: str | Path) -> AnalysisConfig:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("thermal analysis config must be a JSON object")
    equipment = tuple(
        _parse_roi(item) for item in document.get("equipment_rois", [])
    )
    environment = tuple(
        _parse_roi(item) for item in document.get("environment_rois", [])
    )
    if not equipment:
        raise ValueError("at least one equipment ROI is required")
    if any(not roi.roi_id for roi in (*equipment, *environment)):
        raise ValueError("every ROI needs a non-empty id")
    voxel_size = float(document.get("voxel_size_m", 0.1))
    min_points = int(document.get("min_points_per_voxel", 4))
    if voxel_size <= 0.0 or min_points <= 0:
        raise ValueError("voxel size and minimum point count must be positive")
    return AnalysisConfig(
        frame_id=str(document.get("frame_id", "map")),
        voxel_size_m=voxel_size,
        min_points_per_voxel=min_points,
        equipment_rois=equipment,
        environment_rois=environment,
        min_confidence=float(document.get("min_confidence", 0.25)),
        schema_version=int(document.get("schema_version", 1)),
    )


def percentile(values: Sequence[float], percentage: float) -> float:
    if not values:
        raise ValueError("cannot calculate a percentile of no values")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _statistics(values: Sequence[float]) -> dict[str, float | int]:
    return {
        "mean_temperature_c": fmean(values),
        "median_temperature_c": median(values),
        "p90_temperature_c": percentile(values, 90.0),
        "p95_temperature_c": percentile(values, 95.0),
        "max_temperature_c": max(values),
        "point_count": len(values),
    }


def _cell_count(roi: AxisAlignedRoi, size: float) -> int:
    return math.prod(
        max(1, math.ceil((high - low) / size))
        for low, high in zip(roi.minimum, roi.maximum)
    )


def analyze_points(
    points: Iterable[ThermalPoint], config: AnalysisConfig
) -> dict[str, object]:
    """Aggregate one observation without turning unobserved cells into zero."""

    equipment_cells: dict[
        str, dict[tuple[int, int, int], list[float]]
    ] = {roi.roi_id: {} for roi in config.equipment_rois}
    ambient_values: list[float] = []

    for point in points:
        if (
            point.confidence < config.min_confidence
            or not math.isfinite(point.temperature_c)
        ):
            continue
        matched_equipment = False
        for roi in config.equipment_rois:
            if not roi.contains(point.x, point.y, point.z):
                continue
            index = tuple(
                int(math.floor((value - low) / config.voxel_size_m))
                for value, low in zip(
                    (point.x, point.y, point.z), roi.minimum
                )
            )
            equipment_cells[roi.roi_id].setdefault(index, []).append(
                point.temperature_c
            )
            matched_equipment = True
            break
        if matched_equipment:
            continue
        if any(
            roi.contains(point.x, point.y, point.z)
            for roi in config.environment_rois
        ):
            ambient_values.append(point.temperature_c)

    ambient = (
        {
            "mean_temperature_c": fmean(ambient_values),
            "median_temperature_c": median(ambient_values),
            "point_count": len(ambient_values),
        }
        if ambient_values
        else None
    )
    ambient_temperature = (
        float(ambient["median_temperature_c"])
        if ambient is not None
        else None
    )

    equipment_results: list[dict[str, object]] = []
    for roi in config.equipment_rois:
        voxels: list[dict[str, object]] = []
        all_temperatures: list[float] = []
        for index, temperatures in sorted(equipment_cells[roi.roi_id].items()):
            if len(temperatures) < config.min_points_per_voxel:
                continue
            stats = _statistics(temperatures)
            center = [
                roi.minimum[axis]
                + (index[axis] + 0.5) * config.voxel_size_m
                for axis in range(3)
            ]
            p95 = float(stats["p95_temperature_c"])
            voxels.append(
                {
                    "voxel_id": f"{roi.roi_id}:{index[0]}:{index[1]}:{index[2]}",
                    "index": list(index),
                    "center": center,
                    **stats,
                    "delta_p95_c": (
                        p95 - ambient_temperature
                        if ambient_temperature is not None
                        else None
                    ),
                    "valid": True,
                }
            )
            all_temperatures.extend(temperatures)

        total_cells = _cell_count(roi, config.voxel_size_m)
        equipment_stats = _statistics(all_temperatures) if all_temperatures else None
        hottest = (
            max(voxels, key=lambda item: float(item["p95_temperature_c"]))
            if voxels
            else None
        )
        equipment_results.append(
            {
                "equipment_id": roi.roi_id,
                "observed_voxel_count": len(voxels),
                "configured_voxel_count": total_cells,
                "coverage_ratio": len(voxels) / total_cells,
                "statistics": equipment_stats,
                "hottest_voxel_id": (
                    hottest["voxel_id"] if hottest is not None else None
                ),
                "voxels": voxels,
                "thresholds": {
                    "warning_temperature_c": roi.warning_temperature_c,
                    "critical_temperature_c": roi.critical_temperature_c,
                    "warning_delta_c": roi.warning_delta_c,
                },
            }
        )

    return {
        "schema_version": config.schema_version,
        "frame_id": config.frame_id,
        "ambient": ambient,
        "equipment": equipment_results,
    }
