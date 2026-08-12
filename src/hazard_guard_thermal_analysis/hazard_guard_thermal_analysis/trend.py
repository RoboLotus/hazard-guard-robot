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
    minimum_slope_c_per_visit: float = 0.75
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
            "minimum_slope_c_per_visit",
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
        history_window_visits=int(raw.get("history_window_visits", defaults.history_window_visits)),
        min_trend_visits=int(raw.get("min_trend_visits", defaults.min_trend_visits)),
        minimum_rise_c=float(raw.get("minimum_rise_c", defaults.minimum_rise_c)),
        minimum_slope_c_per_visit=float(raw.get("minimum_slope_c_per_visit", defaults.minimum_slope_c_per_visit)),
        minimum_positive_fraction=float(raw.get("minimum_positive_fraction", defaults.minimum_positive_fraction)),
        adaptive_residual_c=float(raw.get("adaptive_residual_c", defaults.adaptive_residual_c)),
        noise_deadband_c=float(raw.get("noise_deadband_c", defaults.noise_deadband_c)),
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


def _evaluate_voxel(
    voxel: Mapping[str, object],
    prior_voxels: Sequence[Mapping[str, object]],
    thresholds: Mapping[str, object],
    config: TrendConfig,
) -> dict[str, object]:
    current_temperature = _number(voxel.get("p95_temperature_c"))
    current_signal, signal_name = _signal(voxel)
    prior_signals = [
        signal
        for signal, prior_name in (_signal(item) for item in prior_voxels)
        if signal is not None and prior_name == signal_name
    ]
    series = (
        prior_signals + ([current_signal] if current_signal is not None else [])
    )[-config.history_window_visits:]
    increments = [later - earlier for earlier, later in zip(series, series[1:])]
    positive_fraction = (
        sum(value > config.noise_deadband_c for value in increments)
        / len(increments)
        if increments
        else 0.0
    )
    total_rise = series[-1] - series[0] if len(series) >= 2 else 0.0
    slope = total_rise / (len(series) - 1) if len(series) >= 2 else 0.0
    trend = (
        len(series) >= config.min_trend_visits
        and total_rise >= config.minimum_rise_c
        and slope >= config.minimum_slope_c_per_visit
        and positive_fraction >= config.minimum_positive_fraction
    )

    warning_temperature = _number(thresholds.get("warning_temperature_c"))
    critical_temperature = _number(thresholds.get("critical_temperature_c"))
    warning_delta = _number(thresholds.get("warning_delta_c"))
    current_delta = _number(voxel.get("delta_p95_c"))
    threshold_exceeded = bool(
        (
            warning_temperature is not None
            and current_temperature is not None
            and current_temperature >= warning_temperature
        )
        or (
            warning_delta is not None
            and current_delta is not None
            and current_delta >= warning_delta
        )
    )
    baseline = median(prior_signals) if prior_signals else None
    residual = (
        current_signal - baseline
        if current_signal is not None and baseline is not None
        else None
    )
    adaptive = threshold_exceeded and (
        residual is None
        or residual >= config.adaptive_residual_c
        or (
            warning_delta is not None
            and current_delta is not None
            and current_delta >= warning_delta
        )
    )
    critical = bool(
        critical_temperature is not None
        and current_temperature is not None
        and current_temperature >= critical_temperature
    )

    if critical:
        status, reason = "critical", "critical_temperature"
    elif trend and adaptive:
        status, reason = "warning", "persistent_trend_and_environment_adjusted_anomaly"
    elif trend:
        status, reason = "watch", "persistent_trend_only"
    elif adaptive:
        status, reason = "watch", "environment_adjusted_anomaly_only"
    else:
        status, reason = "normal", "within_expected_range"

    return {
        "status": status,
        "reason": reason,
        "critical": critical,
        "trend": trend,
        "adaptive": adaptive,
        "signal": signal_name,
        "visit_count": len(series),
        "total_rise_c": round(total_rise, 4),
        "slope_c_per_visit": round(slope, 4),
        "positive_fraction": round(positive_fraction, 4),
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
    summaries: list[dict[str, object]] = []
    equipment_items = result.get("equipment", [])
    if not isinstance(equipment_items, list):
        equipment_items = []

    for equipment in equipment_items:
        if not isinstance(equipment, dict):
            continue
        equipment_id = str(equipment.get("equipment_id", ""))
        historical_voxel_maps = [
            _voxel_by_id(items[equipment_id])
            for items in prior_equipment
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
                item[voxel_id]
                for item in historical_voxel_maps
                if voxel_id in item
            ]
            decision = _evaluate_voxel(voxel, priors, thresholds, config)
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
        "visit_index": len(history) + 1,
        "decision_rule": "critical OR (trend AND adaptive)",
        "config": asdict(config),
        "equipment": summaries,
    }
    return result