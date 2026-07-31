from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def visible_heat_sources(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    sources: Iterable[dict[str, Any]],
    *,
    horizontal_fov_deg: float = 57.0,
    range_min_m: float = 0.0,
    range_max_m: float = 5.0,
) -> list[dict[str, Any]]:
    """Return heat sources inside the simulated thermal camera sector."""

    half_fov = math.radians(horizontal_fov_deg) / 2.0
    visible: list[dict[str, Any]] = []
    for source in sources:
        dx = float(source["x"]) - robot_x
        dy = float(source["y"]) - robot_y
        distance = math.hypot(dx, dy)
        if distance < range_min_m or distance > range_max_m:
            continue
        bearing = math.atan2(dy, dx)
        offset = normalize_angle(bearing - robot_yaw)
        if abs(offset) > half_fov:
            continue
        angular_quality = 1.0 - abs(offset) / max(half_fov, 1e-6)
        distance_quality = 1.0 - distance / range_max_m
        visible.append(
            {
                **source,
                "distance_m": distance,
                "bearing_offset_rad": offset,
                "confidence": max(
                    0.45,
                    min(0.98, 0.62 + 0.2 * angular_quality + 0.16 * distance_quality),
                ),
            }
        )
    return visible
