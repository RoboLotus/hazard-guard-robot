"""Small schema-3 thermal decision rule used by the patrol analyzer."""

from __future__ import annotations

import math
from typing import Mapping, Sequence

from .baseline import EquipmentBaseline


def _number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def evaluate_simple_voxel(
    voxel: Mapping[str, object],
    prior_samples: Sequence[
        tuple[
            Mapping[str, object],
            float | None,
            str | None,
            float | None,
        ]
    ],
    current_time: float | None,
    current_time_source: str | None,
    current_environment_reference_c: float | None,
    *,
    trend_window_visits: int,
    minimum_step_c: float,
    minimum_total_rise_c: float,
    baseline: EquipmentBaseline | None,
    critical_temperature_c: float | None,
    adaptive_delta_c: float,
    p95_valid: bool,
) -> dict[str, object]:
    """Apply ``Critical OR (Trend AND Adaptive)`` to one Voxel.

    Critical always uses the absolute P95. Trend and Adaptive use P95 minus
    the validated environment reference when every required value exists;
    otherwise they fall back to the original absolute-temperature signal.
    """

    voxel_id = str(voxel.get("voxel_id", ""))
    samples: list[tuple[float, float, float | None]] = []
    for prior, timestamp, source, environment_reference in prior_samples:
        value = _number(prior.get("p95_temperature_c"))
        if (
            value is not None
            and timestamp is not None
            and source == current_time_source
        ):
            compensated = (
                value - environment_reference
                if environment_reference is not None
                else None
            )
            samples.append((timestamp, value, compensated))

    current_value = _number(voxel.get("p95_temperature_c"))
    current_compensated = (
        current_value - current_environment_reference_c
        if current_value is not None
        and current_environment_reference_c is not None
        else None
    )
    if current_value is not None and current_time is not None:
        samples.append((current_time, current_value, current_compensated))
    samples = samples[-trend_window_visits:]

    ordered = all(
        later[0] > earlier[0]
        for earlier, later in zip(samples, samples[1:])
    )
    trend_uses_environment = (
        len(samples) == trend_window_visits
        and all(sample[2] is not None for sample in samples)
    )
    values = [
        float(sample[2]) if trend_uses_environment else sample[1]
        for sample in samples
    ]
    increments = [
        later - earlier for earlier, later in zip(values, values[1:])
    ]
    total_rise = (
        values[-1] - values[0]
        if len(values) >= 2 and ordered
        else 0.0
    )
    trend = (
        p95_valid
        and len(samples) == trend_window_visits
        and ordered
        and all(increment >= minimum_step_c for increment in increments)
        and total_rise >= minimum_total_rise_c
    )

    baseline_stats = (
        baseline.for_voxel(voxel_id) if baseline is not None else None
    )
    adaptive_uses_environment = (
        current_compensated is not None
        and baseline_stats is not None
        and baseline_stats.environment_delta_c is not None
    )
    adaptive_current = (
        current_compensated
        if adaptive_uses_environment
        else current_value
    )
    adaptive_baseline = (
        baseline_stats.environment_delta_c
        if adaptive_uses_environment and baseline_stats is not None
        else (
            baseline_stats.temperature_c
            if baseline_stats is not None
            else None
        )
    )
    residual = (
        adaptive_current - adaptive_baseline
        if adaptive_current is not None and adaptive_baseline is not None
        else None
    )
    adaptive = (
        p95_valid
        and residual is not None
        and residual >= adaptive_delta_c
    )
    critical = (
        p95_valid
        and current_value is not None
        and critical_temperature_c is not None
        and current_value >= critical_temperature_c
    )

    if critical:
        status, reason = "critical", "critical_p95_temperature"
    elif trend and adaptive:
        status, reason = "warning", "trend_and_adaptive"
    elif trend:
        status, reason = "watch", "trend_only_recheck"
    elif adaptive:
        status, reason = "watch", "adaptive_only_recheck"
    elif baseline_stats is None:
        status, reason = "normal", "baseline_pending"
    else:
        status, reason = "normal", "within_expected_range"

    return {
        "status": status,
        "reason": reason,
        "critical": critical,
        "critical_p95": critical,
        "critical_max": False,
        "trend": trend,
        "adaptive": adaptive,
        "signal": "voxel_p95_temperature",
        "trend_signal": (
            "p95_minus_environment_reference"
            if trend_uses_environment
            else "p95_temperature"
        ),
        "adaptive_signal": (
            "p95_minus_environment_reference"
            if adaptive_uses_environment
            else "p95_temperature"
        ),
        "environment_compensation_used": (
            trend_uses_environment or adaptive_uses_environment
        ),
        "environment_reference_c": (
            round(current_environment_reference_c, 4)
            if current_environment_reference_c is not None
            else None
        ),
        "compensated_temperature_c": (
            round(current_compensated, 4)
            if current_compensated is not None
            else None
        ),
        "current_temperature_c": (
            round(current_value, 4) if current_value is not None else None
        ),
        "visit_count": len(samples),
        "total_rise_c": round(total_rise, 4),
        "minimum_step_c": minimum_step_c,
        "minimum_rise_threshold_c": minimum_total_rise_c,
        "baseline_temperature_c": (
            round(baseline_stats.temperature_c, 4)
            if baseline_stats is not None
            else None
        ),
        "baseline_environment_delta_c": (
            round(baseline_stats.environment_delta_c, 4)
            if baseline_stats is not None
            and baseline_stats.environment_delta_c is not None
            else None
        ),
        "baseline_residual_c": (
            round(residual, 4) if residual is not None else None
        ),
        "baseline_residual_threshold_c": adaptive_delta_c,
        "baseline_state": (
            baseline_stats.state if baseline_stats is not None else "missing"
        ),
        "time_source": current_time_source,
    }
