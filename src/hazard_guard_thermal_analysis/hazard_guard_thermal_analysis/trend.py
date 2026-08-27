"""Immediate, trend and baseline-adaptive thermal anomaly decisions."""

from __future__ import annotations

import copy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

from .baseline import EquipmentBaseline
from .simple_decision import evaluate_simple_voxel

SEVERITY = {"normal": 0, "watch": 1, "warning": 2, "critical": 3}


@dataclass(frozen=True)
class TrendConfig:
    history_window_visits: int = 8
    min_trend_visits: int = 6
    minimum_positive_fraction: float = 1.0
    minimum_step_c: float = 0.5
    noise_deadband_simulation_floor_c: float = 0.5
    fixed_slope_c_per_hour_enabled: bool = False
    trend_alone_can_trigger_critical: bool = False
    critical_max_persistence_visits: int = 2
    min_hot_cluster_pixels: int = 9
    min_adjacent_hot_voxels: int = 2
    schema_version: int = 2
    # Schema-1 compatibility only. Schema 2 obtains these from approved baselines.
    minimum_rise_c: float = 1.0
    minimum_slope_c_per_hour: float = 2.0
    adaptive_residual_c: float = 1.5
    noise_deadband_c: float = 0.2

    default_adaptive_delta_c: float = 10.0
    environment_reference_enabled: bool = True
    minimum_environment_points: int = 40

    def validate(self) -> None:
        if self.history_window_visits < self.min_trend_visits:
            raise ValueError("history window must include minimum trend visits")
        if self.min_trend_visits < 2:
            raise ValueError("minimum trend visits must be at least two")
        if not 0.0 <= self.minimum_positive_fraction <= 1.0:
            raise ValueError("minimum positive fraction must be between 0 and 1")
        if self.critical_max_persistence_visits < 2:
            raise ValueError("critical Max persistence must require at least two visits")
        if self.min_hot_cluster_pixels < 1 or self.min_adjacent_hot_voxels < 1:
            raise ValueError("spatial confirmation counts must be positive")
        if self.minimum_environment_points < 1:
            raise ValueError(
                "minimum environment reference points must be positive"
            )
        for name in (
            "noise_deadband_simulation_floor_c",
            "minimum_rise_c",
            "minimum_slope_c_per_hour",
            "adaptive_residual_c",
            "minimum_step_c",
            "noise_deadband_c",
            "default_adaptive_delta_c",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")

def load_trend_config(path: str | Path) -> TrendConfig:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("thermal analysis config must be a JSON object")
    schema_version = int(document.get("schema_version", 1))
    if schema_version >= 3:
        raw = document.get("decision", {})
    else:
        raw = document.get("history" if schema_version >= 2 else "trend", {})
    quality = document.get("quality", {})
    statistics = document.get("thermal_statistics", {})
    if not isinstance(raw, Mapping) or not isinstance(quality, Mapping):
        raise ValueError("history/trend and quality config must be JSON objects")
    if not isinstance(statistics, Mapping):
        raise ValueError("thermal_statistics must be a JSON object")
    defaults = TrendConfig(schema_version=schema_version)
    config = TrendConfig(
        history_window_visits=int(raw.get("history_window_visits", defaults.history_window_visits if schema_version >= 2 else 5)),
        min_trend_visits=int(raw.get(
            "trend_window_visits" if schema_version >= 3 else "min_trend_visits",
            defaults.min_trend_visits if schema_version >= 2 else 3,
        )),
        minimum_positive_fraction=float(raw.get("minimum_positive_fraction", defaults.minimum_positive_fraction if schema_version >= 2 else 0.67)),
        minimum_step_c=float(raw.get("minimum_step_c", defaults.minimum_step_c)),
        noise_deadband_simulation_floor_c=float(quality.get("noise_deadband_simulation_floor_c", defaults.noise_deadband_simulation_floor_c)),
        fixed_slope_c_per_hour_enabled=bool(raw.get("fixed_slope_c_per_hour_enabled", schema_version < 2)),
        trend_alone_can_trigger_critical=bool(raw.get("trend_alone_can_trigger_critical", False)),
        critical_max_persistence_visits=int(raw.get("critical_max_persistence_visits", defaults.critical_max_persistence_visits)),
        min_hot_cluster_pixels=int(statistics.get("min_hot_cluster_pixels", defaults.min_hot_cluster_pixels if schema_version >= 2 else 1)),
        min_adjacent_hot_voxels=int(statistics.get("min_adjacent_hot_voxels", defaults.min_adjacent_hot_voxels if schema_version >= 2 else 1)),
        schema_version=schema_version,
        minimum_rise_c=float(raw.get("minimum_total_rise_c" if schema_version >= 3 else "minimum_rise_c", defaults.minimum_rise_c)),
        minimum_slope_c_per_hour=float(raw.get("minimum_slope_c_per_hour", defaults.minimum_slope_c_per_hour)),
        adaptive_residual_c=float(raw.get("adaptive_residual_c", defaults.adaptive_residual_c)),
        noise_deadband_c=float(raw.get("noise_deadband_c", defaults.noise_deadband_c)),
        default_adaptive_delta_c=float(raw.get("default_adaptive_delta_c", defaults.default_adaptive_delta_c)),
        environment_reference_enabled=bool(
            raw.get("environment_reference_enabled", True)
        ),
        minimum_environment_points=int(
            raw.get("minimum_environment_points", statistics.get("min_points_per_roi_for_p95", 40))
        ),
    )
    config.validate()
    return config


def read_history(path: str | Path, limit: int | None = None) -> list[dict]:
    history_path = Path(path).expanduser()
    if not history_path.exists():
        return []
    visits: list[dict] = []
    with history_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict) and isinstance(value.get("equipment"), list):
                visits.append(value)
    return visits[-limit:] if limit is not None else visits


def _number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _levels(value: object) -> tuple[float | None, float | None, float | None]:
    if not isinstance(value, Mapping):
        return None, None, None
    return tuple(_number(value.get(name)) for name in ("watch", "warning", "critical"))  # type: ignore[return-value]


def _environment_reference(
    visit: Mapping[str, object], config: TrendConfig
) -> float | None:
    if not config.environment_reference_enabled:
        return None
    value = visit.get("ambient")
    if not isinstance(value, Mapping):
        return None
    temperature = _number(value.get("median_temperature_c"))
    point_count = _number(value.get("point_count"))
    if (
        temperature is None
        or point_count is None
        or point_count < config.minimum_environment_points
    ):
        return None
    return temperature


def _visit_time_hours(visit: Mapping[str, object]) -> tuple[float | None, str | None]:
    recorded_at = _number(visit.get("recorded_at_unix_sec"))
    if recorded_at is not None:
        return recorded_at / 3600.0, "wall_clock"
    stamp = visit.get("stamp")
    if not isinstance(stamp, Mapping):
        return None, None
    seconds = _number(stamp.get("sec"))
    nanoseconds = _number(stamp.get("nanosec")) or 0.0
    if seconds is None:
        return None, None
    return (seconds + nanoseconds / 1_000_000_000.0) / 3600.0, "ros_stamp"


def _equipment_by_id(visit: Mapping[str, object]) -> dict[str, Mapping[str, object]]:
    items = visit.get("equipment", [])
    if not isinstance(items, Sequence):
        return {}
    return {
        str(item.get("equipment_id")): item
        for item in items
        if isinstance(item, Mapping) and item.get("equipment_id")
    }


def _voxel_by_id(equipment: Mapping[str, object]) -> dict[str, Mapping[str, object]]:
    items = equipment.get("voxels", [])
    if not isinstance(items, Sequence):
        return {}
    return {
        str(item.get("voxel_id")): item
        for item in items
        if isinstance(item, Mapping) and item.get("voxel_id")
    }


def _next_visit_index(history: Sequence[Mapping[str, object]]) -> int:
    indexes: list[int] = []
    for visit in history:
        analysis = visit.get("trend_analysis")
        if isinstance(analysis, Mapping):
            value = _number(analysis.get("visit_index"))
            if value is not None and value >= 1 and value.is_integer():
                indexes.append(int(value))
    return max(indexes) + 1 if indexes else len(history) + 1


def _status_from_levels(value: float | None, levels: tuple[float | None, float | None, float | None]) -> str:
    if value is None:
        return "normal"
    watch, warning, critical = levels
    if critical is not None and value >= critical:
        return "critical"
    if warning is not None and value >= warning:
        return "warning"
    if watch is not None and value >= watch:
        return "watch"
    return "normal"


def _maximum_candidate(candidates: Sequence[tuple[str, str]]) -> tuple[str, str]:
    if not candidates:
        return "normal", "within_expected_range"
    # Prefer the later, more specific channel when severities tie.
    return max(
        enumerate(candidates), key=lambda item: (SEVERITY[item[1][0]], item[0])
    )[1]


def _index(voxel: Mapping[str, object]) -> tuple[int, int, int] | None:
    raw = voxel.get("index")
    if not isinstance(raw, Sequence) or len(raw) != 3:
        return None
    try:
        return tuple(int(value) for value in raw)  # type: ignore[return-value]
    except (TypeError, ValueError):
        return None


def _largest_adjacent_component(voxels: Sequence[Mapping[str, object]], threshold: float | None) -> int:
    if threshold is None:
        return 0
    remaining = {
        index
        for voxel in voxels
        if (index := _index(voxel)) is not None
        and (_number(voxel.get("max_temperature_c")) or -math.inf) >= threshold
    }
    largest = 0
    while remaining:
        stack = [remaining.pop()]
        size = 0
        while stack:
            x, y, z = stack.pop()
            size += 1
            for neighbor in ((x - 1, y, z), (x + 1, y, z), (x, y - 1, z), (x, y + 1, z), (x, y, z - 1), (x, y, z + 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        largest = max(largest, size)
    return largest


def _prior_max_confirmed(
    equipment_id: str,
    voxel_id: str,
    critical_threshold: float | None,
    history: Sequence[Mapping[str, object]],
    required_visits: int,
) -> bool:
    if critical_threshold is None or required_visits < 2:
        return False
    consecutive = 1
    for visit in reversed(history):
        equipment = _equipment_by_id(visit).get(equipment_id)
        voxel = _voxel_by_id(equipment).get(voxel_id) if equipment is not None else None
        if voxel is None or (_number(voxel.get("max_temperature_c")) or -math.inf) < critical_threshold:
            break
        consecutive += 1
        if consecutive >= required_visits:
            return True
    return False


def _trend_decision(
    voxel: Mapping[str, object],
    prior_samples: Sequence[tuple[Mapping[str, object], float | None, str | None]],
    current_time: float | None,
    current_time_source: str | None,
    config: TrendConfig,
    baseline: EquipmentBaseline | None,
    simulated: bool,
) -> dict[str, object]:
    voxel_id = str(voxel.get("voxel_id", ""))
    baseline_stats = baseline.for_voxel(voxel_id) if baseline is not None else None
    series: list[tuple[float, float]] = []
    for prior, timestamp, source in prior_samples:
        value = _number(prior.get("median_temperature_c" if config.schema_version >= 2 else "p95_temperature_c"))
        if value is not None and timestamp is not None and source == current_time_source:
            series.append((timestamp, value))
    current_value = _number(voxel.get("median_temperature_c" if config.schema_version >= 2 else "p95_temperature_c"))
    if current_value is not None and current_time is not None:
        series.append((current_time, current_value))
    series = series[-config.history_window_visits:]
    ordered = all(later[0] > earlier[0] for earlier, later in zip(series, series[1:]))
    values = [sample[1] for sample in series]
    increments = [later - earlier for earlier, later in zip(values, values[1:])]
    if baseline_stats is not None:
        deadband = max(
            3.0 * baseline_stats.sigma_repeat_c,
            baseline_stats.sensor_quantization_c,
            config.noise_deadband_simulation_floor_c if simulated else 0.0,
        )
        minimum_rise = max(3.0 * baseline_stats.sigma_normal_c, 3.0 * baseline_stats.sigma_repeat_c)
        residual_threshold = 3.0 * baseline_stats.sigma_residual_c
        residual = current_value - baseline_stats.temperature_c if current_value is not None else None
        baseline_state = baseline_stats.state
    elif config.schema_version == 1:
        deadband = config.noise_deadband_c
        minimum_rise = config.minimum_rise_c
        residual_threshold = config.adaptive_residual_c
        prior_values = values[:-1]
        historical = sum(prior_values) / len(prior_values) if prior_values else None
        residual = current_value - historical if current_value is not None and historical is not None else None
        baseline_state = "legacy_rolling"
    else:
        deadband = config.noise_deadband_simulation_floor_c if simulated else 0.0
        minimum_rise = None
        residual_threshold = None
        residual = None
        baseline_state = "missing"
    positive_fraction = sum(value > deadband for value in increments) / len(increments) if increments else 0.0
    total_rise = values[-1] - values[0] if len(values) >= 2 and ordered else 0.0
    span = series[-1][0] - series[0][0] if len(series) >= 2 and ordered else 0.0
    slope = total_rise / span if span > 0 else 0.0
    enough_evidence = baseline_stats is not None or config.schema_version == 1
    trend = (
        enough_evidence
        and len(series) >= config.min_trend_visits
        and ordered
        and all(value > deadband for value in increments)
        and positive_fraction >= config.minimum_positive_fraction
        and minimum_rise is not None
        and total_rise >= minimum_rise
        and (not config.fixed_slope_c_per_hour_enabled or slope >= config.minimum_slope_c_per_hour)
    )
    residual_exceeded = residual is not None and residual_threshold is not None and residual >= residual_threshold
    status = "warning" if trend and residual_exceeded else "watch" if trend else "normal"
    reason = "persistent_trend_and_baseline_residual" if status == "warning" else "persistent_trend_only" if status == "watch" else "within_expected_range"
    return {
        "status": status,
        "reason": reason,
        "critical": False,
        "critical_p95": False,
        "critical_max": False,
        "trend": trend,
        "signal": "voxel_median_temperature",
        "visit_count": len(series),
        "total_rise_c": round(total_rise, 4),
        "slope_c_per_hour": round(slope, 4),
        "time_span_hours": round(span, 6),
        "time_source": current_time_source,
        "positive_fraction": round(positive_fraction, 4),
        "noise_deadband_c": round(deadband, 4),
        "minimum_rise_threshold_c": round(minimum_rise, 4) if minimum_rise is not None else None,
        "baseline_residual_c": round(residual, 4) if residual is not None else None,
        "baseline_residual_threshold_c": round(residual_threshold, 4) if residual_threshold is not None else None,
        "baseline_state": baseline_state,
    }


def _immediate_decision(
    equipment: Mapping[str, object],
    history: Sequence[Mapping[str, object]],
    config: TrendConfig,
    baseline: EquipmentBaseline | None,
    sensor_values: Mapping[str, float | None],
    simulated: bool,
) -> tuple[dict[str, object], Mapping[str, object] | None]:
    equipment_id = str(equipment.get("equipment_id", ""))
    statistics = equipment.get("statistics")
    stats = statistics if isinstance(statistics, Mapping) else {}
    p95 = _number(stats.get("p95_temperature_c")) if bool(equipment.get("p95_valid", True)) else None
    peak = _number(stats.get("max_temperature_c"))
    voxels = [item for item in equipment.get("voxels", []) if isinstance(item, Mapping)] if isinstance(equipment.get("voxels"), Sequence) else []
    hottest = max(voxels, key=lambda item: _number(item.get("max_temperature_c")) or -math.inf) if voxels else None
    thresholds = equipment.get("thresholds")
    values = thresholds if isinstance(thresholds, Mapping) else {}
    candidates: list[tuple[str, str]] = []

    absolute = (
        _levels(values.get("simulation_fallback_temperature_c"))
        if simulated and any(level is not None for level in _levels(values.get("simulation_fallback_temperature_c")))
        else (
            _number(values.get("watch_temperature_c")),
            _number(values.get("warning_temperature_c")),
            _number(values.get("critical_temperature_c")),
        )
    )
    p95_status = _status_from_levels(p95, absolute)
    if p95_status != "normal":
        candidates.append((p95_status, f"{p95_status}_p95_temperature"))

    equipment_baseline = baseline.equipment if baseline is not None else None
    baseline_delta = p95 - equipment_baseline.temperature_c if p95 is not None and equipment_baseline is not None else None
    baseline_levels = _levels(values.get("baseline_delta_c"))
    baseline_status = _status_from_levels(baseline_delta, baseline_levels)
    baseline_reason = f"{baseline_status}_approved_baseline_delta"
    if values.get("threshold_mode") == "baseline_primary" and baseline_status in {"warning", "critical"}:
        prior_baseline_delta = None
        if history and equipment_baseline is not None:
            prior_equipment = _equipment_by_id(history[-1]).get(equipment_id)
            prior_statistics = (
                prior_equipment.get("statistics")
                if prior_equipment is not None
                else None
            )
            prior_p95 = (
                _number(prior_statistics.get("p95_temperature_c"))
                if isinstance(prior_statistics, Mapping)
                else None
            )
            if prior_p95 is not None:
                prior_baseline_delta = (
                    prior_p95 - equipment_baseline.temperature_c
                )
        warning_level = baseline_levels[1]
        repeated_warning = (
            prior_baseline_delta is not None
            and warning_level is not None
            and prior_baseline_delta >= warning_level
        )
        exceeds_three_sigma = (
            baseline_delta is not None
            and baseline_delta >= 3.0 * equipment_baseline.sigma_normal_c
        )
        if baseline_status == "warning" and not (repeated_warning or exceeds_three_sigma):
            baseline_status = "watch"
            baseline_reason = "watch_baseline_delta_pending_confirmation"
        elif baseline_status == "critical":
            baseline_status = "warning"
            baseline_reason = "baseline_critical_candidate_requires_corroboration"
    if baseline_status != "normal":
        candidates.append((baseline_status, baseline_reason))

    reference_delta = _number(equipment.get("reference_delta_p95_c"))
    if reference_delta is None and hottest is not None:
        reference_delta = _number(hottest.get("reference_delta_p95_c"))
    reference_levels = (
        _number(values.get("watch_delta_c")),
        _number(values.get("warning_delta_c")),
        _number(values.get("critical_delta_c")),
    )
    reference_status = _status_from_levels(reference_delta, reference_levels)
    if reference_status != "normal":
        candidates.append((reference_status, f"{reference_status}_same_material_reference_delta"))

    air_temperature = _number(sensor_values.get("air_temperature_c"))
    air_delta = p95 - air_temperature if p95 is not None and air_temperature is not None else None
    air_status = _status_from_levels(air_delta, _levels(values.get("air_delta_c")))
    if air_status != "normal":
        candidates.append((air_status, f"{air_status}_air_temperature_delta"))

    oil_temperature = _number(sensor_values.get("oil_temperature_c"))
    oil_status = _status_from_levels(oil_temperature, _levels(values.get("oil_temperature_c")))
    if oil_status != "normal":
        candidates.append((oil_status, f"{oil_status}_oil_temperature"))

    rise_threshold = _number(values.get("critical_rise_between_visits_c"))
    if rise_threshold is not None and p95 is not None and history:
        prior = _equipment_by_id(history[-1]).get(equipment_id)
        prior_stats = prior.get("statistics") if prior is not None else None
        prior_p95 = _number(prior_stats.get("p95_temperature_c")) if isinstance(prior_stats, Mapping) else None
        if prior_p95 is not None and p95 - prior_p95 >= rise_threshold:
            candidates.append(("critical", "critical_rise_between_patrols"))

    critical_threshold = absolute[2]
    max_candidate = peak is not None and critical_threshold is not None and peak >= critical_threshold
    spatial_cluster = int(hottest.get("max_hot_cluster_pixels", 0)) if hottest is not None else 0
    adjacent = _largest_adjacent_component(voxels, critical_threshold)
    persistent = bool(
        hottest is not None
        and _prior_max_confirmed(
            equipment_id,
            str(hottest.get("voxel_id", "")),
            critical_threshold,
            history,
            config.critical_max_persistence_visits,
        )
    )
    max_confirmed = max_candidate and (
        spatial_cluster >= config.min_hot_cluster_pixels
        or adjacent >= config.min_adjacent_hot_voxels
        or persistent
    )
    if max_confirmed:
        candidates.append(("critical", "critical_max_spatially_or_persistently_confirmed"))
    elif max_candidate:
        candidates.append(("warning", "unconfirmed_single_max_candidate"))

    status, reason = _maximum_candidate(candidates)
    if status == "critical" and not bool(values.get("surface_alone_can_trip", True)):
        sensor_critical = oil_status == "critical" or air_status == "critical"
        if not sensor_critical:
            status, reason = "warning", "surface_screening_requires_direct_sensor_confirmation"
    decision = {
        "status": status,
        "reason": reason,
        "critical": status == "critical",
        "critical_p95": p95_status == "critical",
        "critical_max": bool(max_confirmed),
        "max_candidate": bool(max_candidate),
        "max_spatial_cluster_pixels": spatial_cluster,
        "max_adjacent_voxels": adjacent,
        "max_persistently_confirmed": persistent,
        "baseline_delta_c": round(baseline_delta, 4) if baseline_delta is not None else None,
        "reference_delta_c": round(reference_delta, 4) if reference_delta is not None else None,
        "air_delta_c": round(air_delta, 4) if air_delta is not None else None,
        "oil_temperature_c": oil_temperature,
    }
    return decision, hottest


def evaluate_visit(
    visit: Mapping[str, object],
    history: Sequence[Mapping[str, object]],
    config: TrendConfig,
    *,
    baselines: Mapping[str, EquipmentBaseline] | None = None,
    sensor_values: Mapping[str, float | None] | None = None,
    simulated: bool = True,
) -> dict[str, object]:
    """Return a copy enriched with immediate and patrol-to-patrol decisions."""

    config.validate()
    baselines = baselines or {}
    sensor_values = sensor_values or {}
    result = copy.deepcopy(dict(visit))
    prior_visits = list(history)[-(config.history_window_visits - 1):]
    prior_equipment = [_equipment_by_id(item) for item in prior_visits]
    prior_times = [_visit_time_hours(item) for item in prior_visits]
    current_time, current_time_source = _visit_time_hours(result)
    summaries: list[dict[str, object]] = []
    equipment_items = result.get("equipment", [])
    if not isinstance(equipment_items, list):
        equipment_items = []
    prior_environments = [_environment_reference(item, config) for item in prior_visits]

    current_environment = _environment_reference(result, config)
    for equipment in equipment_items:
        if not isinstance(equipment, dict):
            continue
        equipment_id = str(equipment.get("equipment_id", ""))
        baseline = baselines.get(equipment_id)
        thresholds = equipment.get("thresholds")
        threshold_values = (
            thresholds if isinstance(thresholds, Mapping) else {}
        )
        critical_temperature = _number(
            threshold_values.get("critical_temperature_c")
        )
        configured_adaptive_delta = _number(
            threshold_values.get("adaptive_delta_c")
        )
        adaptive_threshold_enabled = threshold_values.get(
            "adaptive_threshold_enabled", True
        )
        if not isinstance(adaptive_threshold_enabled, bool):
            adaptive_threshold_enabled = True
        adaptive_delta = (
            config.default_adaptive_delta_c
            if configured_adaptive_delta is None
            else configured_adaptive_delta
        )
        historical = [
            (
                _voxel_by_id(items[equipment_id]),
                *prior_times[index],
                prior_environments[index],
            )
            for index, items in enumerate(prior_equipment)
            if equipment_id in items
        ]
        statuses: list[str] = []
        for voxel in equipment.get("voxels", []):
            if not isinstance(voxel, dict):
                continue
            voxel_id = str(voxel.get("voxel_id", ""))
            priors = [
                (
                    items[voxel_id],
                    timestamp,
                    source,
                    environment_reference,
                )
                for items, timestamp, source, environment_reference in (
                    historical
                )
                if voxel_id in items
            ]
            if config.schema_version >= 3:
                decision = evaluate_simple_voxel(
                    voxel,
                    priors,
                    current_time,
                    current_time_source,
                    current_environment,
                    trend_window_visits=config.min_trend_visits,
                    minimum_step_c=config.minimum_step_c,
                    minimum_total_rise_c=config.minimum_rise_c,
                    baseline=baseline,
                    critical_temperature_c=critical_temperature,
                    adaptive_delta_c=adaptive_delta,
                    adaptive_threshold_enabled=adaptive_threshold_enabled,
                    p95_valid=bool(equipment.get("p95_valid", True)),
                )
            else:
                decision = _trend_decision(
                    voxel,
                    [
                        (item, timestamp, source)
                        for item, timestamp, source, _ in priors
                    ],
                    current_time,
                    current_time_source,
                    config,
                    baseline,
                    simulated,
                )
            voxel["trend_analysis"] = decision
            statuses.append(str(decision["status"]))

        if config.schema_version < 3:
            immediate, hottest = _immediate_decision(equipment, prior_visits, config, baseline, sensor_values, simulated)
            if hottest is not None and isinstance(hottest, dict):
                existing = hottest.get("trend_analysis")
                if not isinstance(existing, dict) or SEVERITY[immediate["status"]] >= SEVERITY[str(existing.get("status", "normal"))]:
                    merged = dict(existing) if isinstance(existing, dict) else {}
                    merged.update(immediate)
                    hottest["trend_analysis"] = merged
        statuses = [
            str(voxel.get("trend_analysis", {}).get("status", "normal"))
            for voxel in equipment.get("voxels", [])
            if isinstance(voxel, Mapping) and isinstance(voxel.get("trend_analysis"), Mapping)
        ]
        status = max(statuses, key=lambda name: SEVERITY.get(name, 0)) if statuses else "normal"
        equipment["trend_status"] = status
        equipment["trend_voxel_counts"] = {name: statuses.count(name) for name in SEVERITY}
        summaries.append({"equipment_id": equipment_id, "status": status, "voxel_counts": dict(equipment["trend_voxel_counts"])})

    decision_config = asdict(config)
    if config.schema_version >= 3:
        decision_config.pop("min_hot_cluster_pixels", None)
        decision_config.pop("min_adjacent_hot_voxels", None)
        decision_config["spatial_cluster_gate_enabled"] = False
    result["trend_analysis"] = {
        "schema_version": config.schema_version,
        "visit_index": _next_visit_index(history),
        "decision_rule": "Absolute Critical always; per-equipment adaptive mode adds environment-compensated Trend AND Adaptive; one signal means watch/recheck",
        "config": decision_config,
        "equipment": summaries,
    }
    # Preserve the acquisition origin in the persisted/published visit payload.
    # The mission manager builds its approval incident from this top-level field,
    # so omitting it made simulated thermal incidents look like physical ones.
    result["simulated"] = bool(simulated)
    return result
