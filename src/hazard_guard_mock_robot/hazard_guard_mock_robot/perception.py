from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any

from hazard_guard_sensor_config import TMC160B


def equipment_id_from_heat_source(source: dict[str, Any]) -> str | None:
    source_name = str(source.get("source") or "")
    if not source_name.startswith("gazebo:"):
        return None
    equipment_id = source_name.split(":", 1)[1].strip()
    return equipment_id or None


def heat_source_temperature(
    source: dict[str, Any],
    incident_temperatures_c: dict[str, float],
) -> float:
    equipment_id = equipment_id_from_heat_source(source)
    if equipment_id and equipment_id in incident_temperatures_c:
        return float(incident_temperatures_c[equipment_id])
    return float(source["temperature_c"])


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_planar_point(
    x: float,
    y: float,
    z: float,
    *,
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Apply a planar quaternion transform to a point."""

    quaternion_x, quaternion_y, quaternion_z, quaternion_w = rotation
    yaw = math.atan2(
        2.0
        * (
            quaternion_w * quaternion_z
            + quaternion_x * quaternion_y
        ),
        1.0 - 2.0 * (quaternion_y**2 + quaternion_z**2),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        translation[0] + cosine * x - sine * y,
        translation[1] + sine * x + cosine * y,
        translation[2] + z,
    )


def visible_heat_sources(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    sources: Iterable[dict[str, Any]],
    *,
    horizontal_fov_deg: float = TMC160B.horizontal_fov_deg,
    range_min_m: float = 0.0,
    range_max_m: float = TMC160B.visualization_range_m,
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
                    min(
                        0.98,
                        0.62
                        + 0.2 * angular_quality
                        + 0.16 * distance_quality,
                    ),
                ),
            }
        )
    return visible
