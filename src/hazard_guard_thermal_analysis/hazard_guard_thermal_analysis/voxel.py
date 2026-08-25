from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from statistics import fmean, median
from typing import Iterable, Mapping, Sequence

from .projection import ThermalPoint


@dataclass(frozen=True)
class EquipmentTrendThresholds:
    minimum_rise_c: float | None = None
    minimum_slope_c_per_hour: float | None = None


@dataclass(frozen=True)
class AxisAlignedRoi:
    roi_id: str
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    display_name: str = ""
    warning_temperature_c: float | None = None
    critical_temperature_c: float | None = None
    warning_delta_c: float | None = None
    watch_temperature_c: float | None = None
    watch_delta_c: float | None = None
    adaptive_delta_c: float | None = None
    adaptive_threshold_enabled: bool = True
    critical_delta_c: float | None = None
    trend: EquipmentTrendThresholds | None = None
    threshold_mode: str = "absolute"
    reference_roi_id: str | None = None
    simulation_watch_temperature_c: float | None = None
    simulation_warning_temperature_c: float | None = None
    simulation_critical_temperature_c: float | None = None
    baseline_watch_delta_c: float | None = None
    baseline_warning_delta_c: float | None = None
    baseline_critical_delta_c: float | None = None
    air_watch_delta_c: float | None = None
    air_warning_delta_c: float | None = None
    air_critical_delta_c: float | None = None
    oil_watch_temperature_c: float | None = None
    oil_warning_temperature_c: float | None = None
    oil_critical_temperature_c: float | None = None
    critical_rise_between_visits_c: float | None = None
    surface_alone_can_trip: bool = True

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
    reference_rois: tuple[AxisAlignedRoi, ...] = ()
    min_points_per_roi_for_p95: int = 1
    recommended_points_per_roi_for_p95: int = 1
    min_hot_cluster_pixels: int = 1
    min_adjacent_hot_voxels: int = 1


def _vector3(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain three numbers")
    if len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result  # type: ignore[return-value]


def _optional_float(value: Mapping[str, object], name: str) -> float | None:
    raw = value.get(name)
    if raw is None:
        return None
    result = float(raw)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return result


def _optional_bool(
    value: Mapping[str, object], name: str, *, default: bool
) -> bool:
    raw = value.get(name, default)
    if not isinstance(raw, bool):
        raise ValueError(f"{name} must be a boolean")
    return raw


def _levels(
    value: Mapping[str, object], name: str
) -> tuple[float | None, float | None, float | None]:
    raw = value.get(name)
    if raw is None:
        return None, None, None
    if not isinstance(raw, Mapping):
        raise ValueError(f"{name} must be a JSON object")
    levels = tuple(_optional_float(raw, key) for key in ("watch", "warning", "critical"))
    configured = [item for item in levels if item is not None]
    if any(lower >= upper for lower, upper in zip(configured, configured[1:])):
        raise ValueError(f"{name} thresholds must increase")
    return levels  # type: ignore[return-value]


def _parse_roi(value: object) -> AxisAlignedRoi:
    if not isinstance(value, Mapping):
        raise ValueError("ROI entries must be objects")
    minimum = _vector3(value.get("min"), "ROI min")
    maximum = _vector3(value.get("max"), "ROI max")
    if any(low >= high for low, high in zip(minimum, maximum)):
        raise ValueError("each ROI min value must be smaller than max")

    raw_trend = value.get("trend")
    if raw_trend is not None and not isinstance(raw_trend, Mapping):
        raise ValueError("ROI trend must be a JSON object")

    def optional_trend_float(name: str) -> float | None:
        if not isinstance(raw_trend, Mapping):
            return None
        return _optional_float(raw_trend, name)

    watch_temperature = _optional_float(value, "watch_temperature_c")
    warning_temperature = _optional_float(value, "warning_temperature_c")
    critical_temperature = _optional_float(value, "critical_temperature_c")
    watch_delta = _optional_float(value, "watch_delta_c")
    warning_delta = _optional_float(value, "warning_delta_c")
    critical_delta = _optional_float(value, "critical_delta_c")

    def validate_levels(name: str, levels: Sequence[float | None]) -> None:
        configured = [level for level in levels if level is not None]
        if any(lower >= upper for lower, upper in zip(configured, configured[1:])):
            raise ValueError(f"ROI {name} thresholds must increase")

    validate_levels(
        "temperature", (watch_temperature, warning_temperature, critical_temperature)
    )
    validate_levels("delta", (watch_delta, warning_delta, critical_delta))

    simulation = _levels(value, "simulation_fallback_temperature_c")
    baseline = _levels(value, "baseline_delta_c")
    air = _levels(value, "air_delta_c")
    oil = _levels(value, "oil_temperature_c")
    trend = EquipmentTrendThresholds(
        minimum_rise_c=optional_trend_float("minimum_rise_c"),
        minimum_slope_c_per_hour=optional_trend_float(
            "minimum_slope_c_per_hour"
        ),
    )
    if trend.minimum_rise_c is None and trend.minimum_slope_c_per_hour is None:
        trend = None

    reference_roi_id = str(value.get("reference_roi_id", "")).strip() or None
    threshold_mode = str(value.get("threshold_mode", "absolute")).strip()
    if threshold_mode not in {"absolute", "baseline_primary", "screening"}:
        raise ValueError("unsupported threshold_mode")

    return AxisAlignedRoi(
        roi_id=str(value.get("id", "")).strip(),
        minimum=minimum,
        maximum=maximum,
        display_name=(
            str(value.get("name", value.get("display_name", ""))).strip()
            or str(value.get("id", "")).strip()
        ),
        watch_temperature_c=watch_temperature,
        warning_temperature_c=warning_temperature,
        critical_temperature_c=critical_temperature,
        adaptive_delta_c=_optional_float(value, "adaptive_delta_c"),
        adaptive_threshold_enabled=_optional_bool(
            value, "adaptive_threshold_enabled", default=True
        ),
        watch_delta_c=watch_delta,
        warning_delta_c=warning_delta,
        critical_delta_c=critical_delta,
        trend=trend,
        threshold_mode=threshold_mode,
        reference_roi_id=reference_roi_id,
        simulation_watch_temperature_c=simulation[0],
        simulation_warning_temperature_c=simulation[1],
        simulation_critical_temperature_c=simulation[2],
        baseline_watch_delta_c=baseline[0],
        baseline_warning_delta_c=baseline[1],
        baseline_critical_delta_c=baseline[2],
        air_watch_delta_c=air[0],
        air_warning_delta_c=air[1],
        air_critical_delta_c=air[2],
        oil_watch_temperature_c=oil[0],
        oil_warning_temperature_c=oil[1],
        oil_critical_temperature_c=oil[2],
        critical_rise_between_visits_c=_optional_float(
            value, "critical_rise_between_visits_c"
        ),
        surface_alone_can_trip=bool(value.get("surface_alone_can_trip", True)),
    )


def load_config(path: str | Path) -> AnalysisConfig:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("thermal analysis config must be a JSON object")
    equipment = tuple(_parse_roi(item) for item in document.get("equipment_rois", []))
    environment = tuple(
        _parse_roi(item) for item in document.get("environment_rois", [])
    )
    references = tuple(_parse_roi(item) for item in document.get("reference_rois", []))
    if not equipment:
        raise ValueError("at least one equipment ROI is required")
    if any(not roi.roi_id for roi in (*equipment, *environment, *references)):
        raise ValueError("every ROI needs a non-empty id")

    schema_version = int(document.get("schema_version", 1))
    statistics = document.get("thermal_statistics", {})
    quality = document.get("quality", {})
    spatial = document.get("spatial", {})
    if not isinstance(statistics, Mapping) or not isinstance(quality, Mapping):
        raise ValueError("thermal_statistics and quality must be objects")
    if not isinstance(spatial, Mapping):
        raise ValueError("spatial must be an object")

    voxel_size = float(
        spatial.get("voxel_size_simulation_m", document.get("voxel_size_m", 0.1))
    )
    min_points = int(
        statistics.get("min_points_per_voxel", document.get("min_points_per_voxel", 4))
    )
    min_p95 = int(statistics.get("min_points_per_roi_for_p95", min_points))
    recommended_p95 = int(
        statistics.get("recommended_points_per_roi_for_p95", min_p95)
    )
    min_cluster = int(statistics.get("min_hot_cluster_pixels", 1))
    min_adjacent = int(statistics.get("min_adjacent_hot_voxels", 1))
    min_confidence = float(
        quality.get("min_confidence_simulation_fallback", document.get("min_confidence", 0.25))
    )
    if (
        voxel_size <= 0.0
        or min_points <= 0
        or min_p95 <= 0
        or recommended_p95 < min_p95
        or min_cluster <= 0
        or min_adjacent <= 0
        or not 0.0 <= min_confidence <= 1.0
    ):
        raise ValueError("thermal analysis quality values are invalid")
    known_references = {roi.roi_id for roi in references}
    for roi in equipment:
        if roi.reference_roi_id and roi.reference_roi_id not in known_references:
            raise ValueError(f"unknown reference ROI {roi.reference_roi_id!r}")

    return AnalysisConfig(
        frame_id=str(document.get("frame_id", "map")),
        voxel_size_m=voxel_size,
        min_points_per_voxel=min_points,
        equipment_rois=equipment,
        environment_rois=environment,
        min_confidence=min_confidence,
        schema_version=schema_version,
        reference_rois=references,
        min_points_per_roi_for_p95=min_p95,
        recommended_points_per_roi_for_p95=recommended_p95,
        min_hot_cluster_pixels=min_cluster,
        min_adjacent_hot_voxels=min_adjacent,
    )


def apply_equipment_settings(
    config: AnalysisConfig, document: Mapping[str, object]
) -> AnalysisConfig:
    """Apply Web UI equipment changes while preserving advanced policy fields."""

    raw_equipment = document.get("equipment")
    if not isinstance(raw_equipment, Sequence) or isinstance(raw_equipment, (str, bytes)):
        raise ValueError("equipment settings must contain an equipment list")
    existing = {roi.roi_id: roi for roi in config.equipment_rois}
    configured: list[AxisAlignedRoi] = []
    seen: set[str] = set()
    for raw in raw_equipment:
        if not isinstance(raw, Mapping):
            raise ValueError("equipment settings entries must be objects")
        if not bool(raw.get("enabled", True)):
            continue
        equipment_id = str(raw.get("id", "")).strip()
        display_name = str(raw.get("display_name", "")).strip()
        if not equipment_id or not display_name:
            raise ValueError("enabled equipment needs an id and display name")
        if equipment_id in seen:
            raise ValueError(f"duplicate equipment id {equipment_id!r}")
        seen.add(equipment_id)
        raw_roi = raw.get("roi")
        if not isinstance(raw_roi, Mapping):
            raise ValueError(f"equipment {equipment_id!r} needs an ROI")
        minimum = _vector3(raw_roi.get("min"), "equipment ROI min")
        maximum = _vector3(raw_roi.get("max"), "equipment ROI max")
        if any(low >= high for low, high in zip(minimum, maximum)):
            raise ValueError("each equipment ROI min value must be smaller than max")
        critical = _optional_float(raw, "critical_temperature_c")
        adaptive = _optional_float(raw, "adaptive_delta_c")
        adaptive_enabled = _optional_bool(
            raw, "adaptive_threshold_enabled", default=True
        )
        if critical is None or adaptive is None:
            raise ValueError(f"equipment {equipment_id!r} needs both thresholds")
        base = existing.get(equipment_id)
        if base is None:
            base = AxisAlignedRoi(
                roi_id=equipment_id,
                minimum=minimum,
                maximum=maximum,
                display_name=display_name,
                critical_temperature_c=critical,
                adaptive_delta_c=adaptive,
                adaptive_threshold_enabled=adaptive_enabled,
            )
        else:
            base = replace(
                base,
                minimum=minimum,
                maximum=maximum,
                display_name=display_name,
                critical_temperature_c=critical,
                adaptive_delta_c=adaptive,
                adaptive_threshold_enabled=adaptive_enabled,
            )
        configured.append(base)
    clearance_m = 0.03
    for index, first in enumerate(configured):
        for second in configured[index + 1 :]:
            conflict = all(
                first.maximum[axis] + clearance_m > second.minimum[axis]
                and second.maximum[axis] + clearance_m > first.minimum[axis]
                for axis in range(3)
            )
            if conflict:
                raise ValueError(
                    "equipment ROIs overlap or are closer than "
                    f"{clearance_m:.2f} m: {first.roi_id!r}, {second.roi_id!r}"
                )
    return replace(config, equipment_rois=tuple(configured))


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


def _largest_pixel_cluster(points: Sequence[ThermalPoint], threshold: float | None) -> int:
    if threshold is None:
        return 0
    remaining = {
        (int(point.pixel_u), int(point.pixel_v))
        for point in points
        if point.temperature_c >= threshold and point.pixel_u >= 0 and point.pixel_v >= 0
    }
    largest = 0
    while remaining:
        stack = [remaining.pop()]
        size = 0
        while stack:
            u, v = stack.pop()
            size += 1
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    if du == 0 and dv == 0:
                        continue
                    neighbor = (u + du, v + dv)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        stack.append(neighbor)
        largest = max(largest, size)
    return largest


def _thresholds(roi: AxisAlignedRoi) -> dict[str, object]:
    return {
        "threshold_mode": roi.threshold_mode,
        "watch_temperature_c": roi.watch_temperature_c,
        "warning_temperature_c": roi.warning_temperature_c,
        "critical_temperature_c": roi.critical_temperature_c,
        "adaptive_delta_c": roi.adaptive_delta_c,
        "adaptive_threshold_enabled": roi.adaptive_threshold_enabled,
        "watch_delta_c": roi.watch_delta_c,
        "warning_delta_c": roi.warning_delta_c,
        "critical_delta_c": roi.critical_delta_c,
        "reference_roi_id": roi.reference_roi_id,
        "simulation_fallback_temperature_c": {
            "watch": roi.simulation_watch_temperature_c,
            "warning": roi.simulation_warning_temperature_c,
            "critical": roi.simulation_critical_temperature_c,
        },
        "baseline_delta_c": {
            "watch": roi.baseline_watch_delta_c,
            "warning": roi.baseline_warning_delta_c,
            "critical": roi.baseline_critical_delta_c,
        },
        "air_delta_c": {
            "watch": roi.air_watch_delta_c,
            "warning": roi.air_warning_delta_c,
            "critical": roi.air_critical_delta_c,
        },
        "oil_temperature_c": {
            "watch": roi.oil_watch_temperature_c,
            "warning": roi.oil_warning_temperature_c,
            "critical": roi.oil_critical_temperature_c,
        },
        "critical_rise_between_visits_c": roi.critical_rise_between_visits_c,
        "surface_alone_can_trip": roi.surface_alone_can_trip,
        "trend": (
            {
                "minimum_rise_c": roi.trend.minimum_rise_c,
                "minimum_slope_c_per_hour": roi.trend.minimum_slope_c_per_hour,
            }
            if roi.trend is not None
            else None
        ),
    }


def analyze_points(
    points: Iterable[ThermalPoint], config: AnalysisConfig, *, simulated: bool = True
) -> dict[str, object]:
    """Aggregate one observation without turning unobserved cells into zero."""

    equipment_cells: dict[str, dict[tuple[int, int, int], list[ThermalPoint]]] = {
        roi.roi_id: {} for roi in config.equipment_rois
    }
    reference_values: dict[str, list[float]] = {
        roi.roi_id: [] for roi in config.reference_rois
    }
    ambient_values: list[float] = []

    for point in points:
        if point.confidence < config.min_confidence or not math.isfinite(point.temperature_c):
            continue
        for roi in config.reference_rois:
            if roi.contains(point.x, point.y, point.z):
                reference_values[roi.roi_id].append(point.temperature_c)
        matched_equipment = False
        for roi in config.equipment_rois:
            if not roi.contains(point.x, point.y, point.z):
                continue
            index = tuple(
                int(math.floor((value - low) / config.voxel_size_m))
                for value, low in zip((point.x, point.y, point.z), roi.minimum)
            )
            equipment_cells[roi.roi_id].setdefault(index, []).append(point)
            matched_equipment = True
            break
        if not matched_equipment and any(
            roi.contains(point.x, point.y, point.z) for roi in config.environment_rois
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
        float(ambient["median_temperature_c"]) if ambient is not None else None
    )
    references = {
        roi_id: {
            "median_temperature_c": median(values),
            "mean_temperature_c": fmean(values),
            "point_count": len(values),
        }
        for roi_id, values in reference_values.items()
        if values
    }

    equipment_results: list[dict[str, object]] = []
    for roi in config.equipment_rois:
        voxels: list[dict[str, object]] = []
        all_temperatures: list[float] = []
        reference = references.get(roi.reference_roi_id or "")
        reference_temperature = (
            float(reference["median_temperature_c"]) if reference is not None else None
        )
        pixel_critical = (
            roi.simulation_critical_temperature_c
            if simulated and roi.simulation_critical_temperature_c is not None
            else roi.critical_temperature_c
        )
        for index, samples in sorted(equipment_cells[roi.roi_id].items()):
            if len(samples) < config.min_points_per_voxel:
                continue
            temperatures = [sample.temperature_c for sample in samples]
            stats = _statistics(temperatures)
            center = [
                roi.minimum[axis] + (index[axis] + 0.5) * config.voxel_size_m
                for axis in range(3)
            ]
            p95 = float(stats["p95_temperature_c"])
            pixel_count = len(
                {
                    (int(sample.pixel_u), int(sample.pixel_v))
                    for sample in samples
                    if sample.pixel_u >= 0 and sample.pixel_v >= 0
                }
            )
            voxels.append(
                {
                    "voxel_id": f"{roi.roi_id}:{index[0]}:{index[1]}:{index[2]}",
                    "index": list(index),
                    "center": center,
                    **stats,
                    "ambient_delta_p95_c": (
                        p95 - ambient_temperature if ambient_temperature is not None else None
                    ),
                    "reference_delta_p95_c": (
                        p95 - reference_temperature if reference_temperature is not None else None
                    ),
                    "delta_p95_c": (
                        p95 - reference_temperature
                        if reference_temperature is not None
                        else (
                            p95 - ambient_temperature
                            if config.schema_version == 1 and ambient_temperature is not None
                            else None
                        )
                    ),
                    "radiometric_pixel_count": pixel_count,
                    "max_hot_cluster_pixels": _largest_pixel_cluster(
                        samples, pixel_critical
                    ),
                    "valid": True,
                }
            )
            all_temperatures.extend(temperatures)

        total_cells = _cell_count(roi, config.voxel_size_m)
        equipment_stats = _statistics(all_temperatures) if all_temperatures else None
        p95_valid = len(all_temperatures) >= config.min_points_per_roi_for_p95
        quality_flags = []
        if all_temperatures and not p95_valid:
            quality_flags.append("insufficient_samples_for_p95")
        if len(all_temperatures) < config.recommended_points_per_roi_for_p95:
            quality_flags.append("below_recommended_p95_samples")
        if roi.reference_roi_id and reference_temperature is None:
            quality_flags.append("reference_roi_unavailable")
        hottest = (
            max(voxels, key=lambda item: float(item["p95_temperature_c"]))
            if voxels
            else None
        )
        equipment_results.append(
            {
                "equipment_id": roi.roi_id,
                "display_name": roi.display_name or roi.roi_id,
                "observed_voxel_count": len(voxels),
                "configured_voxel_count": total_cells,
                "coverage_ratio": len(voxels) / total_cells,
                "statistics": equipment_stats,
                "p95_valid": p95_valid,
                "quality_flags": quality_flags,
                "reference_temperature_c": reference_temperature,
                "hottest_voxel_id": hottest["voxel_id"] if hottest is not None else None,
                "voxels": voxels,
                "thresholds": _thresholds(roi),
            }
        )

    return {
        "schema_version": config.schema_version,
        "frame_id": config.frame_id,
        "ambient": ambient,
        "references": references,
        "quality": {
            "min_points_per_voxel": config.min_points_per_voxel,
            "min_points_per_roi_for_p95": config.min_points_per_roi_for_p95,
            "recommended_points_per_roi_for_p95": config.recommended_points_per_roi_for_p95,
            "min_hot_cluster_pixels": config.min_hot_cluster_pixels,
            "min_adjacent_hot_voxels": config.min_adjacent_hot_voxels,
        },
        "equipment": equipment_results,
    }
