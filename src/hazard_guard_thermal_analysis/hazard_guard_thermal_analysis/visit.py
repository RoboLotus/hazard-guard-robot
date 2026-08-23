"""Build one stable thermal history sample from a completed patrol lap."""

from __future__ import annotations

import copy
from statistics import fmean, median
from typing import Mapping


_METRICS = (
    "mean_temperature_c",
    "median_temperature_c",
    "p90_temperature_c",
    "p95_temperature_c",
    "max_temperature_c",
)


def _number(value: object) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _representative_statistics(items: list[dict[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for metric in _METRICS:
        values = [value for item in items if (value := _number(item.get(metric))) is not None]
        if values:
            result[metric] = median(values)
    counts = [int(item.get("point_count", 0)) for item in items]
    result["point_count"] = int(median(counts)) if counts else 0
    result["frame_count"] = len(items)
    return result


class PatrolVisitAccumulator:
    """Collect focused camera frames and emit one value per voxel per lap."""

    def __init__(self) -> None:
        self.active = False
        self.focus_equipment_id: str | None = None
        self.frame_count = 0
        self._ambient: list[dict[str, object]] = []
        self._references: dict[str, list[dict[str, object]]] = {}
        self._equipment: dict[str, dict[str, object]] = {}
        self._schema_version = 1
        self._frame_id = "map"
        self._quality: dict[str, object] = {}

    def start(self) -> None:
        self.active = True
        self.focus_equipment_id = None
        self.frame_count = 0
        self._ambient.clear()
        self._references.clear()
        self._equipment.clear()
        self._schema_version = 1
        self._frame_id = "map"
        self._quality.clear()

    def focus(self, equipment_id: str | None) -> None:
        if not self.active:
            self.start()
        self.focus_equipment_id = equipment_id or None

    def add(self, result: Mapping[str, object]) -> None:
        if not self.active or not self.focus_equipment_id:
            return
        self.frame_count += 1
        self._schema_version = int(result.get("schema_version", self._schema_version))
        self._frame_id = str(result.get("frame_id", self._frame_id))
        quality = result.get("quality")
        if isinstance(quality, Mapping):
            self._quality = copy.deepcopy(dict(quality))
        ambient = result.get("ambient")
        if isinstance(ambient, Mapping):
            self._ambient.append(dict(ambient))
        references = result.get("references")
        if isinstance(references, Mapping):
            for reference_id, reference in references.items():
                if isinstance(reference, Mapping):
                    self._references.setdefault(str(reference_id), []).append(dict(reference))

        equipment_items = result.get("equipment", [])
        if not isinstance(equipment_items, list):
            return
        for equipment in equipment_items:
            if not isinstance(equipment, Mapping):
                continue
            equipment_id = str(equipment.get("equipment_id", ""))
            if not equipment_id or equipment_id != self.focus_equipment_id:
                continue
            voxels = equipment.get("voxels", [])
            if not isinstance(voxels, list) or not voxels:
                continue
            entry = self._equipment.setdefault(
                equipment_id,
                {"template": copy.deepcopy(dict(equipment)), "frames": 0, "statistics": [], "voxels": {}},
            )
            entry["frames"] = int(entry["frames"]) + 1
            frame_statistics = equipment.get("statistics")
            if isinstance(frame_statistics, Mapping):
                entry["statistics"].append(dict(frame_statistics))
            observations = entry["voxels"]
            for voxel in voxels:
                if isinstance(voxel, Mapping) and voxel.get("voxel_id"):
                    observations.setdefault(str(voxel["voxel_id"]), []).append(copy.deepcopy(dict(voxel)))

    @property
    def equipment_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._equipment))

    def _ambient_result(self) -> dict[str, object] | None:
        if not self._ambient:
            return None
        medians = [value for item in self._ambient if (value := _number(item.get("median_temperature_c"))) is not None]
        means = [value for item in self._ambient if (value := _number(item.get("mean_temperature_c"))) is not None]
        counts = [int(item.get("point_count", 0)) for item in self._ambient]
        if not medians:
            return None
        return {
            "mean_temperature_c": fmean(means) if means else median(medians),
            "median_temperature_c": median(medians),
            "point_count": int(median(counts)) if counts else 0,
            "frame_count": len(medians),
        }

    def _reference_results(self) -> dict[str, dict[str, object]]:
        return {
            reference_id: _representative_statistics(items)
            for reference_id, items in self._references.items()
            if items
        }

    @staticmethod
    def _voxel_result(observations: list[dict[str, object]], schema_version: int) -> dict[str, object]:
        representative = copy.deepcopy(observations[-1])
        representative.update(_representative_statistics(observations))
        for name in ("ambient_delta_p95_c", "reference_delta_p95_c", "delta_p95_c"):
            values = [value for item in observations if (value := _number(item.get(name))) is not None]
            representative[name] = median(values) if values else None
        for name in ("radiometric_pixel_count", "max_hot_cluster_pixels"):
            values = [int(item.get(name, 0)) for item in observations]
            representative[name] = int(median(values)) if values else 0
        if schema_version >= 2 and representative.get("reference_delta_p95_c") is None:
            representative["delta_p95_c"] = None
        representative.pop("trend_analysis", None)
        return representative

    def finalize(self, stamp: Mapping[str, object] | None = None) -> dict[str, object]:
        ambient = self._ambient_result()
        references = self._reference_results()
        equipment_results: list[dict[str, object]] = []
        min_p95 = int(self._quality.get("min_points_per_roi_for_p95", 1))
        recommended_p95 = int(self._quality.get("recommended_points_per_roi_for_p95", min_p95))
        for equipment_id in sorted(self._equipment):
            entry = self._equipment[equipment_id]
            template = copy.deepcopy(entry["template"])
            observations = entry["voxels"]
            voxels = [self._voxel_result(items, self._schema_version) for _, items in sorted(observations.items()) if items]
            template["voxels"] = voxels
            template["observed_voxel_count"] = len(voxels)
            template["coverage_ratio"] = len(voxels) / max(1, int(template.get("configured_voxel_count", 1)))
            template["capture_frame_count"] = int(entry["frames"])
            frame_statistics = entry["statistics"]
            if frame_statistics:
                template["statistics"] = _representative_statistics(frame_statistics)
            stats = template.get("statistics")
            point_count = int(stats.get("point_count", 0)) if isinstance(stats, Mapping) else 0
            template["p95_valid"] = point_count >= min_p95
            flags = [item for item in template.get("quality_flags", []) if item not in {"insufficient_samples_for_p95", "below_recommended_p95_samples"}]
            if point_count < min_p95:
                flags.append("insufficient_samples_for_p95")
            if point_count < recommended_p95:
                flags.append("below_recommended_p95_samples")
            template["quality_flags"] = flags
            reference_id = template.get("thresholds", {}).get("reference_roi_id") if isinstance(template.get("thresholds"), Mapping) else None
            reference = references.get(str(reference_id)) if reference_id else None
            p95 = _number(stats.get("p95_temperature_c")) if isinstance(stats, Mapping) else None
            reference_temperature = _number(reference.get("median_temperature_c")) if isinstance(reference, Mapping) else None
            template["reference_temperature_c"] = reference_temperature
            template["reference_delta_p95_c"] = p95 - reference_temperature if p95 is not None and reference_temperature is not None else None
            template["hottest_voxel_id"] = max(voxels, key=lambda item: float(item["p95_temperature_c"]))["voxel_id"] if voxels else None
            equipment_results.append(template)

        result: dict[str, object] = {
            "schema_version": self._schema_version,
            "frame_id": self._frame_id,
            "ambient": ambient,
            "references": references,
            "quality": copy.deepcopy(self._quality),
            "equipment": equipment_results,
            "visit_capture": {
                "schema_version": 2 if self._schema_version >= 2 else 1,
                "frame_count": self.frame_count,
                "equipment_count": len(equipment_results),
                "aggregation": "temporal_median_of_focused_frames",
            },
        }
        if stamp is not None:
            result["stamp"] = dict(stamp)
        return result
