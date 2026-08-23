from __future__ import annotations

from typing import Any


SEVERITY_ORDER = {"normal": 0, "watch": 1, "warning": 2, "critical": 3}


def thermal_observations(
    payload: dict[str, Any], *, mission_id: str | None
) -> list[dict[str, Any]]:
    trend = payload.get("trend_analysis", {})
    visit_index = trend.get("visit_index") if isinstance(trend, dict) else None
    frame_id = str(payload.get("frame_id") or "map")
    observations = []
    for equipment in payload.get("equipment", []):
        if not isinstance(equipment, dict):
            continue
        equipment_id = str(equipment.get("equipment_id") or "").strip()
        if not equipment_id:
            continue
        severity = str(equipment.get("trend_status") or "normal").lower()
        candidates = []
        for voxel in equipment.get("voxels", []):
            if not isinstance(voxel, dict):
                continue
            decision = voxel.get("trend_analysis", {})
            voxel_severity = str(
                decision.get("status") if isinstance(decision, dict) else "normal"
            ).lower()
            center = voxel.get("center", [])
            if len(center) < 3:
                center = [0.0, 0.0, 0.0]
            temperature = float(
                voxel.get("max_temperature_c", voxel.get("p95_temperature_c", 0.0))
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
        visit_suffix = f"visit-{visit_index}" if isinstance(visit_index, int) else "visit"
        observations.append(
            {
                "incident_id": f"thermal:{equipment_id}:{visit_suffix}",
                "detection_id": f"thermal-{equipment_id}",
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
