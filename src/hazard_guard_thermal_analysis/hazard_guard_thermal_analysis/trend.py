"""Long-term trend decisions for fixed equipment voxels."""

from __future__ import annotations

import copy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from statistics import median
from typing import Mapping, Sequence

SEVERITY = {"normal": 0, "watch": 1, "warning": 2, "critical": 3}


@dataclass(frozen=True)
class TrendConfig:
    history_window_visits: int = 5
    min_trend_visits: int = 3
    minimum_rise_c: float = 2.0
    minimum_slope_c_per_hour: float = 2.0
    minimum_positive_fraction: float = 0.67
    adaptive_residual_c: float = 1.5
    noise_deadband_c: float = 0.2

    def validate(self) -> None:
        if self.history_window_visits < self.min_trend_visits:
            raise ValueError("history window must include minimum trend visits")
        if self.min_trend_visits < 2:
            raise ValueError("minimum trend visits must be at least two")
        if not 0.0 <= self.minimum_positive_fraction <= 1.0:
            raise ValueError("minimum positive fraction must be between 0 and 1")
        for name in (
            "minimum_rise_c",
            "minimum_slope_c_per_hour",
            "adaptive_residual_c",
            "noise_deadband_c",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")


def load_trend_config(path: str | Path) -> TrendConfig:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    raw = document.get("trend", {})
    if not isinstance(raw, Mapping):
        raise ValueError("trend config must be a JSON object")
    defaults = TrendConfig()
    config = TrendConfig(
        history_window_visits=int(
            raw.get(
                "history_window_visits", defaults.history_window_visits
            )
        ),
        min_trend_visits=int(
            raw.get("min_trend_visits", defaults.min_trend_visits)
        ),
        minimum_rise_c=float(
            raw.get("minimum_rise_c", defaults.minimum_rise_c)
        ),
        minimum_slope_c_per_hour=float(
            raw.get(
                "minimum_slope_c_per_hour",
                defaults.minimum_slope_c_per_hour,
            )
        ),
        minimum_positive_fraction=float(
            raw.get(
                "minimum_positive_fraction",
                defaults.minimum_positive_fraction,
            )
        ),
        adaptive_residual_c=float(
            raw.get("adaptive_residual_c", defaults.adaptive_residual_c)
        ),
        noise_deadband_c=float(
            raw.get("noise_deadband_c", defaults.noise_deadband_c)
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


def _signal(voxel: Mapping[str, object]) -> tuple[float | None, str]:
    corrected = _number(voxel.get("delta_p95_c"))
    if corrected is not None:
        return corrected, "ambient_corrected_p95"
    return _number(voxel.get("p95_temperature_c")), "p95"


def _visit_time_hours(visit: Mapping[str, object]) -> tuple[float | None, str | None]:
    """Return a stable time axis without mixing wall and ROS simulation clocks."""

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


def _equipment_by_id(visit: Mapping[str, object]) -> dict[str, Mapping]:
    items = visit.get("equipment", [])
    if not isinstance(items, Sequence):
        return {}
    return {
        str(item.get("equipment_id")): item
        for item in items
        if isinstance(item, Mapping) and item.get("equipment_id")
    }


def _voxel_by_id(equipment: Mapping[str, object]) -> dict[str, Mapping]:
    items = equipment.get("voxels", [])
    if not isinstance(items, Sequence):
        return {}
    return {
        str(item.get("voxel_id")): item
        for item in items
        if isinstance(item, Mapping) and item.get("voxel_id")
    }


def _next_visit_index(history: Sequence[Mapping[str, object]]) -> int:
    """Continue the persisted patrol sequence independently of window size."""

    indexes: list[int] = []
    for visit in history:
        analysis = visit.get("trend_analysis")
        if not isinstance(analysis, Mapping):
            continue
        value = _number(analysis.get("visit_index"))
        if value is not None and value >= 1 and value.is_integer():
            indexes.append(int(value))
    return (max(indexes) + 1) if indexes else (len(history) + 1)


def _evaluate_voxel(
    voxel: Mapping[str, object],
    prior_voxels: Sequence[
        tuple[Mapping[str, object], float | None, str | None]
    ],
    current_time_hours: float | None,
    current_time_source: str | None,
    thresholds: Mapping[str, object],
    config: TrendConfig,
) -> dict[str, object]:
    current_temperature = _number(voxel.get("p95_temperature_c"))
    current_max_temperature = _number(voxel.get("max_temperature_c"))
    current_signal, signal_name = _signal(voxel)
    prior_signals = [
        signal
        for signal, prior_name in (
            _signal(item) for item, _, _ in prior_voxels
        )
        if signal is not None and prior_name == signal_name
    ]

    timed_series = [
        (timestamp, signal)
        for item, timestamp, source in prior_voxels
        for signal, prior_name in (_signal(item),)
        if (
            timestamp is not None
            and source == current_time_source
            and signal is not None
            and prior_name == signal_name
        )
    ]
    if current_signal is not None and current_time_hours is not None:
        timed_series.append((current_time_hours, current_signal))
    timed_series = timed_series[-config.history_window_visits:]
    timestamps_increase = all(
        later[0] > earlier[0]
        for earlier, later in zip(timed_series, timed_series[1:])
    )
    timed_values = [sample[1] for sample in timed_series]
    increments = [
        later - earlier
        for earlier, later in zip(timed_values, timed_values[1:])
    ]
    positive_fraction = (
        sum(value > config.noise_deadband_c for value in increments)
        / len(increments)
        if increments
        else 0.0
    )
    time_span_hours = (
        timed_series[-1][0] - timed_series[0][0]
        if len(timed_series) >= 2 and timestamps_increase
        else 0.0
    )
    total_rise = (
        timed_values[-1] - timed_values[0]
        if len(timed_values) >= 2 and timestamps_increase
        else 0.0
    )
    slope_per_hour = (
        total_rise / time_span_hours if time_span_hours > 0.0 else 0.0
    )
    raw_trend_thresholds = thresholds.get("trend", {})
    trend_thresholds = (
        raw_trend_thresholds
        if isinstance(raw_trend_thresholds, Mapping)
        else {}
    )
    minimum_rise = _number(trend_thresholds.get("minimum_rise_c"))
    minimum_slope = _number(
        trend_thresholds.get("minimum_slope_c_per_hour")
    )
    if minimum_rise is None:
        minimum_rise = config.minimum_rise_c
    if minimum_slope is None:
        minimum_slope = config.minimum_slope_c_per_hour
    trend = (
        len(timed_series) >= config.min_trend_visits
        and timestamps_increase
        and total_rise >= minimum_rise
        and slope_per_hour >= minimum_slope
        and positive_fraction >= config.minimum_positive_fraction
    )

    watch_temperature = _number(thresholds.get("watch_temperature_c"))
    warning_temperature = _number(thresholds.get("warning_temperature_c"))
    critical_temperature = _number(thresholds.get("critical_temperature_c"))
    watch_delta = _number(thresholds.get("watch_delta_c"))
    warning_delta = _number(thresholds.get("warning_delta_c"))
    critical_delta = _number(thresholds.get("critical_delta_c"))
    current_delta = _number(voxel.get("delta_p95_c"))
    explicit_watch_levels = (
        watch_temperature is not None or watch_delta is not None
    )

    def exceeds(value: float | None, threshold: float | None) -> bool:
        return bool(
            value is not None
            and threshold is not None
            and value >= threshold
        )

    if explicit_watch_levels:
        watch_threshold_exceeded = (
            exceeds(current_temperature, watch_temperature)
            or exceeds(current_delta, watch_delta)
        )
        warning_threshold_exceeded = (
            exceeds(current_temperature, warning_temperature)
            or exceeds(current_delta, warning_delta)
        )
    else:
        # Backward compatibility: legacy warning fields represented the
        # adaptive Watch level and required a trend before Warning.
        watch_threshold_exceeded = (
            exceeds(current_temperature, warning_temperature)
            or exceeds(current_delta, warning_delta)
        )
        warning_threshold_exceeded = False

    baseline = median(prior_signals) if prior_signals else None
    residual = (
        current_signal - baseline
        if current_signal is not None and baseline is not None
        else None
    )
    adaptive = watch_threshold_exceeded and (
        residual is None
        or residual >= config.adaptive_residual_c
        or (
            (watch_delta if explicit_watch_levels else warning_delta)
            is not None
            and current_delta is not None
            and current_delta
            >= (watch_delta if explicit_watch_levels else warning_delta)
        )
    )
    watch_triggered = (
        watch_threshold_exceeded if explicit_watch_levels else adaptive
    )
    critical_p95 = exceeds(current_temperature, critical_temperature)
    critical_max = exceeds(current_max_temperature, critical_temperature)
    critical_delta_exceeded = exceeds(current_delta, critical_delta)
    critical = critical_p95 or critical_max or critical_delta_exceeded

    if critical:
        if critical_p95 and critical_max:
            reason = "critical_p95_and_max_temperature"
        elif critical_p95:
            reason = "critical_p95_temperature"
        elif critical_max:
            reason = "critical_max_temperature"
        else:
            reason = "critical_ambient_delta"
        if critical_delta_exceeded and (critical_p95 or critical_max):
            reason += "_and_ambient_delta"
        status = "critical"
    elif warning_threshold_exceeded:
        warning_temperature_exceeded = exceeds(
            current_temperature, warning_temperature
        )
        warning_delta_exceeded = exceeds(current_delta, warning_delta)
        if warning_temperature_exceeded and warning_delta_exceeded:
            reason = "warning_p95_temperature_and_ambient_delta"
        elif warning_temperature_exceeded:
            reason = "warning_p95_temperature"
        else:
            reason = "warning_ambient_delta"
        status = "warning"
    elif trend and watch_triggered:
        status, reason = "warning", "persistent_trend_and_watch_anomaly"
    elif trend:
        status, reason = "watch", "persistent_trend_only"
    elif watch_triggered:
        status, reason = "watch", "watch_threshold_exceeded"
    else:
        status, reason = "normal", "within_expected_range"

    return {
        "status": status,
        "reason": reason,
        "critical": critical,
        "critical_p95": critical_p95,
        "critical_max": critical_max,
        "critical_delta": critical_delta_exceeded,
        "trend": trend,
        "adaptive": adaptive,
        "watch_threshold_exceeded": watch_threshold_exceeded,
        "warning_threshold_exceeded": warning_threshold_exceeded,
        "signal": signal_name,
        "visit_count": len(timed_series),
        "total_rise_c": round(total_rise, 4),
        "slope_c_per_hour": round(slope_per_hour, 4),
        "time_span_hours": round(time_span_hours, 6),
        "time_source": current_time_source,
        "positive_fraction": round(positive_fraction, 4),
        "minimum_rise_threshold_c": round(minimum_rise, 4),
        "minimum_slope_threshold_c_per_hour": round(minimum_slope, 4),
        "historical_baseline_c": round(baseline, 4) if baseline is not None else None,
        "adaptive_residual_c": round(residual, 4) if residual is not None else None,
    }


def evaluate_visit(
    visit: Mapping[str, object],
    history: Sequence[Mapping[str, object]],
    config: TrendConfig,
) -> dict[str, object]:
    """Return a copy enriched with per-voxel and equipment decisions."""

    config.validate()
    result = copy.deepcopy(dict(visit))
    prior_visits = list(history)[-(config.history_window_visits - 1):]
    prior_equipment = [_equipment_by_id(item) for item in prior_visits]
    prior_times = [_visit_time_hours(item) for item in prior_visits]
    current_time_hours, current_time_source = _visit_time_hours(result)
    summaries: list[dict[str, object]] = []
    equipment_items = result.get("equipment", [])
    if not isinstance(equipment_items, list):
        equipment_items = []

    for equipment in equipment_items:
        if not isinstance(equipment, dict):
            continue
        equipment_id = str(equipment.get("equipment_id", ""))
        historical_voxel_samples = [
            (_voxel_by_id(items[equipment_id]), *prior_times[index])
            for index, items in enumerate(prior_equipment)
            if equipment_id in items
        ]
        raw_thresholds = equipment.get("thresholds", {})
        thresholds = raw_thresholds if isinstance(raw_thresholds, Mapping) else {}
        statuses: list[str] = []
        for voxel in equipment.get("voxels", []):
            if not isinstance(voxel, dict):
                continue
            voxel_id = str(voxel.get("voxel_id", ""))
            priors = [
                (items[voxel_id], timestamp, source)
                for items, timestamp, source in historical_voxel_samples
                if voxel_id in items
            ]
            decision = _evaluate_voxel(
                voxel,
                priors,
                current_time_hours,
                current_time_source,
                thresholds,
                config,
            )
            voxel["trend_analysis"] = decision
            statuses.append(str(decision["status"]))

        status = max(statuses, key=SEVERITY.get) if statuses else "normal"
        equipment["trend_status"] = status
        equipment["trend_voxel_counts"] = {
            name: statuses.count(name) for name in SEVERITY
        }
        summaries.append(
            {
                "equipment_id": equipment_id,
                "status": status,
                "voxel_counts": dict(equipment["trend_voxel_counts"]),
            }
        )

    result["trend_analysis"] = {
        "schema_version": 1,
        "visit_index": _next_visit_index(history),
        "decision_rule": (
            "critical OR warning_threshold OR "
            "(trend AND watch_threshold)"
        ),
        "config": asdict(config),
        "equipment": summaries,
    }
    return result
