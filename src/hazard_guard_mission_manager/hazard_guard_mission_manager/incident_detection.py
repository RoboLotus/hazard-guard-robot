from __future__ import annotations

import hashlib
import math
from typing import Any


SEVERITY_ORDER = {"normal": 0, "watch": 1, "warning": 2, "critical": 3}


def thermal_visit_index(payload: dict[str, Any]) -> int:
    trend = payload.get("trend_analysis")
    if not isinstance(trend, dict):
        raise ValueError("trend_analysis가 객체가 아닙니다")
    visit_index = trend.get("visit_index")
    if not isinstance(visit_index, int) or isinstance(visit_index, bool):
        raise ValueError("visit_index가 유효한 정수가 아닙니다")
    if visit_index < 1:
        raise ValueError("visit_index는 1 이상이어야 합니다")
    return visit_index


def _finite_number(value: Any, *, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field}가 숫자가 아닙니다")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}가 숫자가 아닙니다") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field}가 유한한 숫자가 아닙니다")
    return number


def thermal_observations(
    payload: dict[str, Any], *, mission_id: str | None
) -> list[dict[str, Any]]:
    if not isinstance(payload, dict):
        raise ValueError("열화상 결과가 객체가 아닙니다")
    visit_index = thermal_visit_index(payload)
    frame_id = str(payload.get("frame_id") or "map")
    equipment_items = payload.get("equipment")
    if not isinstance(equipment_items, list):
        raise ValueError("equipment가 배열이 아닙니다")
    observations = []
    for equipment in equipment_items:
        if not isinstance(equipment, dict):
            raise ValueError("equipment 항목이 객체가 아닙니다")
        equipment_id = str(equipment.get("equipment_id") or "").strip()
        if not equipment_id:
            continue
        severity = str(equipment.get("trend_status") or "normal").lower()
        candidates = []
        voxel_items = equipment.get("voxels", [])
        if not isinstance(voxel_items, list):
            raise ValueError("voxels가 배열이 아닙니다")
        for voxel in voxel_items:
            if not isinstance(voxel, dict):
                raise ValueError("voxel 항목이 객체가 아닙니다")
            decision = voxel.get("trend_analysis", {})
            voxel_severity = str(
                decision.get("status") if isinstance(decision, dict) else "normal"
            ).lower()
            center = voxel.get("center", [])
            if not isinstance(center, (list, tuple)) or len(center) < 3:
                center = [0.0, 0.0, 0.0]
            center = [
                _finite_number(center[index], field=f"center[{index}]")
                for index in range(3)
            ]
            temperature = _finite_number(
                voxel.get(
                    "max_temperature_c",
                    voxel.get("p95_temperature_c", 0.0),
                ),
                field="temperature",
            )
            candidates.append(
                (
                    SEVERITY_ORDER.get(voxel_severity, 0),
                    temperature,
                    center,
                )
            )
        _rank, temperature, center = max(
            candidates,
            default=(SEVERITY_ORDER.get(severity, 0), 0.0, [0.0, 0.0, 0.0]),
            key=lambda item: (item[0], item[1]),
        )
        mission_suffix = str(mission_id or "unassigned").strip() or "unassigned"
        detection_key = hashlib.sha256(
            f"{mission_suffix}|{equipment_id}|{visit_index}".encode("utf-8")
        ).hexdigest()[:32]
        observations.append(
            {
                "incident_id": (
                    f"thermal:{mission_suffix}:{equipment_id}:visit-{visit_index}"
                ),
                "detection_id": f"thermal:{detection_key}",
                "mission_id": mission_id or "",
                "equipment_id": equipment_id,
                "source": "thermal_trend",
                "severity": severity,
                "frame_id": frame_id,
                "x": float(center[0]),
                "y": float(center[1]),
                "z": float(center[2]),
                "temperature_c": temperature,
                "confidence": 1.0,
                "simulated": bool(payload.get("simulated", False)),
                "message": str(equipment.get("reason") or severity),
            }
        )
    return observations
