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


class PatrolVisitAccumulator:
    """Collect focused camera frames and emit one value per voxel per lap."""

    def __init__(self) -> None:
        self.active = False
        self.focus_equipment_id: str | None = None
        self.frame_count = 0
        self._ambient: list[dict[str, object]] = []
        self._equipment: dict[str, dict[str, object]] = {}

    def start(self) -> None:
        self.active = True
        self.focus_equipment_id = None
        self.frame_count = 0
        self._ambient.clear()
        self._equipment.clear()

    def focus(self, equipment_id: str | None) -> None:
        if not self.active:
            self.start()
        self.focus_equipment_id = equipment_id or None

    def add(self, result: Mapping[str, object]) -> None:
        if not self.active or not self.focus_equipment_id:
            return
        self.frame_count += 1
        ambient = result.get("ambient")
        if isinstance(ambient, Mapping):
            self._ambient.append(dict(ambient))

        equipment_items = result.get("equipment", [])
        if not isinstance(equipment_items, list):
            return
        for equipment in equipment_items:
            if not isinstance(equipment, Mapping):
                continue
            equipment_id = str(equipment.get("equipment_id", ""))
            if not equipment_id or (
                self.focus_equipment_id
                and equipment_id != self.focus_equipment_id
            ):
                continue
            voxels = equipment.get("voxels", [])
            if not isinstance(voxels, list) or not voxels:
                continue
            entry = self._equipment.setdefault(
                equipment_id,
                {
                    "template": copy.deepcopy(dict(equipment)),
                    "frames": 0,
                    "statistics": [],
                    "voxels": {},
                },
            )
            entry["frames"] = int(entry["frames"]) + 1
            frame_statistics = equipment.get("statistics")
            if isinstance(frame_statistics, Mapping):
                statistics = entry["statistics"]
                assert isinstance(statistics, list)
                statistics.append(dict(frame_statistics))
            observations = entry["voxels"]
            assert isinstance(observations, dict)
            for voxel in voxels:
                if not isinstance(voxel, Mapping) or not voxel.get("voxel_id"):
                    continue
                observations.setdefault(str(voxel["voxel_id"]), []).append(
                    copy.deepcopy(dict(voxel))
                )

    @property
    def equipment_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._equipment))

    def _ambient_result(self) -> dict[str, object] | None:
        if not self._ambient:
            return None
        medians = [
            value
            for item in self._ambient
            if (value := _number(item.get("median_temperature_c"))) is not None
        ]
        means = [
            value
            for item in self._ambient
            if (value := _number(item.get("mean_temperature_c"))) is not None
        ]
        counts = [int(item.get("point_count", 0)) for item in self._ambient]
        if not medians:
            return None
        return {
            "mean_temperature_c": fmean(means) if means else median(medians),
            "median_temperature_c": median(medians),
            "point_count": int(median(counts)) if counts else 0,
            "frame_count": len(medians),
        }

    @staticmethod
    def _voxel_result(
        observations: list[dict[str, object]],
        ambient_temperature: float | None,
    ) -> dict[str, object]:
        representative = copy.deepcopy(observations[-1])
        for metric in _METRICS:
            values = [
                value
                for item in observations
                if (value := _number(item.get(metric))) is not None
            ]
            if values:
                representative[metric] = median(values)
        counts = [int(item.get("point_count", 0)) for item in observations]
        representative["point_count"] = int(median(counts)) if counts else 0
        representative["frame_count"] = len(observations)
        p95 = _number(representative.get("p95_temperature_c"))
        representative["delta_p95_c"] = (
            p95 - ambient_temperature
            if p95 is not None and ambient_temperature is not None
            else None
        )
        representative.pop("trend_analysis", None)
        return representative

    def finalize(self, stamp: Mapping[str, object] | None = None) -> dict[str, object]:
        ambient = self._ambient_result()
        ambient_temperature = (
            _number(ambient.get("median_temperature_c"))
            if ambient is not None
            else None
        )
        equipment_results: list[dict[str, object]] = []
        for equipment_id in sorted(self._equipment):
            entry = self._equipment[equipment_id]
            template = copy.deepcopy(entry["template"])
            assert isinstance(template, dict)
            observations = entry["voxels"]
            assert isinstance(observations, dict)
            voxels = [
                self._voxel_result(items, ambient_temperature)
                for _, items in sorted(observations.items())
                if items
            ]
            template["voxels"] = voxels
            template["observed_voxel_count"] = len(voxels)
            template["coverage_ratio"] = (
                len(voxels) / max(1, int(template.get("configured_voxel_count", 1)))
            )
            template["capture_frame_count"] = int(entry["frames"])
            frame_statistics = entry["statistics"]
            assert isinstance(frame_statistics, list)
            if frame_statistics:
                representative_statistics: dict[str, object] = {}
                for metric in _METRICS:
                    values = [
                        value
                        for item in frame_statistics
                        if (value := _number(item.get(metric))) is not None
                    ]
                    if values:
                        representative_statistics[metric] = median(values)
                counts = [
                    int(item.get("point_count", 0)) for item in frame_statistics
                ]
                representative_statistics["point_count"] = (
                    int(median(counts)) if counts else 0
                )
                representative_statistics["frame_count"] = len(frame_statistics)
                template["statistics"] = representative_statistics
            template["hottest_voxel_id"] = (
                max(voxels, key=lambda item: float(item["p95_temperature_c"]))[
                    "voxel_id"
                ]
                if voxels
                else None
            )
            equipment_results.append(template)

        result: dict[str, object] = {
            "schema_version": 1,
            "frame_id": "map",
            "ambient": ambient,
            "equipment": equipment_results,
            "visit_capture": {
                "schema_version": 1,
                "frame_count": self.frame_count,
                "equipment_count": len(equipment_results),
                "aggregation": "temporal_median_of_focused_frames",
            },
        }
        if stamp is not None:
            result["stamp"] = dict(stamp)
        return result
