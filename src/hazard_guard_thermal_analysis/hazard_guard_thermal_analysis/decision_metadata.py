from __future__ import annotations


def effective_threshold_range(
    result: object, equipment_id: str
) -> tuple[float | None, float | None]:
    """Return the observed min/max voxel threshold for one equipment item."""

    if not isinstance(result, dict):
        return None, None
    for equipment in result.get("equipment", []):
        if not isinstance(equipment, dict):
            continue
        if str(equipment.get("equipment_id", "")) != equipment_id:
            continue
        candidates = []
        for voxel in equipment.get("voxels", []):
            if not isinstance(voxel, dict):
                continue
            decision = voxel.get("trend_analysis", {})
            if not isinstance(decision, dict):
                continue
            threshold = decision.get("effective_adaptive_threshold_c")
            if isinstance(threshold, (int, float)):
                candidates.append(float(threshold))
        if candidates:
            return min(candidates), max(candidates)
        return None, None
    return None, None
