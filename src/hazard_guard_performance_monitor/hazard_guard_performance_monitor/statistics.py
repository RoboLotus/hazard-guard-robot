from __future__ import annotations

import math
import statistics
from collections.abc import Iterable


def percentile(values: Iterable[float], probability: float) -> float | None:
    """Return a linearly interpolated percentile for finite values."""

    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return None
    probability = min(1.0, max(0.0, float(probability)))
    position = (len(finite) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] + (finite[upper] - finite[lower]) * fraction


def summarize_values(values: Iterable[float]) -> dict[str, float | int | None]:
    finite = [float(value) for value in values if math.isfinite(value)]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p95": None,
            "max": None,
            "stddev": None,
        }
    return {
        "count": len(finite),
        "mean": round(statistics.fmean(finite), 3),
        "median": round(statistics.median(finite), 3),
        "p95": round(float(percentile(finite, 0.95)), 3),
        "max": round(max(finite), 3),
        "stddev": round(statistics.pstdev(finite), 3),
    }
