"""Persistent commissioning samples for approved thermal baselines."""

from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import median
from typing import Mapping, Sequence


def _number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _mapping_items(value: object) -> list[Mapping[str, object]]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return []
    return [item for item in value if isinstance(item, Mapping)]


def _atomic_json(path: Path, document: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(
            document,
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class BaselineCollector:
    """Collect stable completed visits until every configured ROI is ready."""

    def __init__(
        self,
        collection_path: str | Path,
        baseline_path: str | Path,
        equipment_ids: Sequence[str],
        *,
        minimum_valid_visits: int = 10,
        minimum_environment_points: int = 40,
        stability_recovery_visits: int = 3,
    ) -> None:
        if minimum_valid_visits < 2:
            raise ValueError("minimum_valid_visits must be at least two")
        if minimum_environment_points < 1:
            raise ValueError("minimum_environment_points must be positive")
        if stability_recovery_visits < 1:
            raise ValueError("stability_recovery_visits must be positive")
        self.collection_path = Path(collection_path).expanduser()
        self.baseline_path = Path(baseline_path).expanduser()
        self.equipment_ids = tuple(sorted(set(equipment_ids)))
        if not self.equipment_ids:
            raise ValueError("at least one equipment id is required")
        self.minimum_valid_visits = minimum_valid_visits
        self.minimum_environment_points = minimum_environment_points
        self._equipment: dict[str, dict[str, object]] = {}
        self.stability_recovery_visits = stability_recovery_visits
        self._load()

    def _load(self) -> None:
        if not self.collection_path.exists():
            return
        document = json.loads(
            self.collection_path.read_text(encoding="utf-8")
        )
        if not isinstance(document, Mapping):
            raise ValueError("baseline collection must be a JSON object")
        raw_equipment = document.get("equipment", {})
        if not isinstance(raw_equipment, Mapping):
            raise ValueError("baseline collection equipment must be an object")
        for equipment_id in self.equipment_ids:
            raw = raw_equipment.get(equipment_id)
            if not isinstance(raw, Mapping):
                continue
            samples = self._valid_samples(raw.get("samples"))
            raw_voxels = raw.get("voxels", {})
            voxels: dict[str, list[dict[str, float]]] = {}
            if isinstance(raw_voxels, Mapping):
                for voxel_id, values in raw_voxels.items():
                    valid = self._valid_samples(values)
                    if valid:
                        voxels[str(voxel_id)] = valid
            raw_recent = raw.get("recent_visit_times", [])
            recent_values = (
                raw_recent
                if isinstance(raw_recent, Sequence)
                and not isinstance(raw_recent, (str, bytes))
                else []
            )
            recent_limit = self.stability_recovery_visits - 1
            recent_visit_times = [
                value
                for item in recent_values
                if (value := _number(item)) is not None
            ]
            self._equipment[equipment_id] = {
                "samples": samples[: self.minimum_valid_visits],
                "voxels": voxels,
                "paused": bool(raw.get("paused", False)),
                "stable_streak": max(0, int(raw.get("stable_streak", 0))),
                "excluded": [
                    dict(item)
                    for item in _mapping_items(raw.get("excluded"))
                ][-100:],
                "recent_visit_times": (
                    recent_visit_times[-recent_limit:]
                    if recent_limit > 0
                    else []
                ),
            }

    @staticmethod
    def _valid_samples(value: object) -> list[dict[str, float]]:
        result: list[dict[str, float]] = []
        for item in _mapping_items(value):
            temperature = _number(item.get("temperature_c"))
            delta = _number(item.get("environment_delta_c"))
            if temperature is None or delta is None:
                continue
            sample = {
                "temperature_c": temperature,
                "environment_delta_c": delta,
            }
            recorded_at = _number(item.get("recorded_at_unix_sec"))
            if recorded_at is not None:
                sample["recorded_at_unix_sec"] = recorded_at
            result.append(sample)
        return result

    def _document(self) -> dict[str, object]:
        return {
            "schema_version": 2,
            "stability_recovery_visits": self.stability_recovery_visits,
            "minimum_valid_visits": self.minimum_valid_visits,
            "minimum_environment_points": (
                self.minimum_environment_points
            ),
            "equipment": self._equipment,
        }

    def _save(self) -> None:
        _atomic_json(self.collection_path, self._document())

    def counts(self) -> dict[str, int]:
        return {
            equipment_id: len(
                self._equipment.get(equipment_id, {}).get("samples", [])
            )
            for equipment_id in self.equipment_ids
        }

    @property
    def ready(self) -> bool:
        return all(
            count >= self.minimum_valid_visits
            and not bool(
                self._equipment.get(equipment_id, {}).get("paused", False)
            )
            for equipment_id, count in self.counts().items()
        )

    def recovery_status(self) -> dict[str, dict[str, object]]:
        return {
            equipment_id: {
                "paused": bool(entry.get("paused", False)),
                "stable_streak": int(entry.get("stable_streak", 0)),
                "excluded_count": len(entry.get("excluded", [])),
            }
            for equipment_id in self.equipment_ids
            if (entry := self._equipment.get(equipment_id)) is not None
        }

    def progress_message(self) -> str:
        parts = []
        for equipment_id, count in self.counts().items():
            entry = self._equipment.get(equipment_id, {})
            suffix = ""
            if bool(entry.get("paused", False)):
                suffix = (
                    ", recovery "
                    f"{int(entry.get('stable_streak', 0))}/"
                    f"{self.stability_recovery_visits}"
                )
            parts.append(
                f"{equipment_id}={count}/{self.minimum_valid_visits}{suffix}"
            )
        progress = ", ".join(parts)
        state = "ready for approval" if self.ready else "collecting"
        return f"Thermal baseline {state}: {progress}"

    @staticmethod
    def _append_exclusion(
        entry: dict[str, object],
        reason: str,
        sample: Mapping[str, object] | None = None,
        recorded_at: float | None = None,
    ) -> None:
        exclusions = entry.setdefault("excluded", [])
        if not isinstance(exclusions, list):
            exclusions = []
            entry["excluded"] = exclusions
        record: dict[str, object] = {"reason": reason}
        if sample is not None:
            for key in (
                "temperature_c",
                "environment_delta_c",
                "recorded_at_unix_sec",
            ):
                value = _number(sample.get(key))
                if value is not None:
                    record[key] = value
        if "recorded_at_unix_sec" not in record and recorded_at is not None:
            record["recorded_at_unix_sec"] = recorded_at
        exclusions.append(record)
        del exclusions[:-100]

    def _exclude_recent_samples(
        self,
        entry: dict[str, object],
        count: int,
        recent_visit_times: Sequence[float] = (),
    ) -> int:
        samples = entry.get("samples")
        if not isinstance(samples, list) or count < 1:
            return 0
        if recent_visit_times:
            recent = set(recent_visit_times)
            removed = [
                sample
                for sample in samples
                if isinstance(sample, Mapping)
                and _number(sample.get("recorded_at_unix_sec")) in recent
            ]
            samples[:] = [
                sample
                for sample in samples
                if not isinstance(sample, Mapping)
                or _number(sample.get("recorded_at_unix_sec")) not in recent
            ]
        else:
            removed = samples[-count:]
            if removed:
                del samples[-len(removed):]
        if not removed:
            return 0
        for sample in removed:
            if isinstance(sample, Mapping):
                self._append_exclusion(
                    entry,
                    "trend_recent_window",
                    sample=sample,
                )

        removed_times = {
            timestamp
            for sample in removed
            if isinstance(sample, Mapping)
            and (
                timestamp := _number(
                    sample.get("recorded_at_unix_sec")
                )
            )
            is not None
        }
        voxel_entries = entry.get("voxels")
        if isinstance(voxel_entries, dict):
            for voxel_id, values in list(voxel_entries.items()):
                if not isinstance(values, list):
                    del voxel_entries[voxel_id]
                    continue
                if removed_times:
                    values[:] = [
                        sample
                        for sample in values
                        if not isinstance(sample, Mapping)
                        or _number(
                            sample.get("recorded_at_unix_sec")
                        )
                        not in removed_times
                    ]
                else:
                    del values[-min(len(values), len(removed)):]
                if not values:
                    del voxel_entries[voxel_id]
        return len(removed)

    def observe(self, visit: Mapping[str, object]) -> dict[str, object]:
        """Add one stable equipment observation from a completed visit."""

        ambient = visit.get("ambient")
        if not isinstance(ambient, Mapping):
            return {
                "accepted": [],
                "paused": [],
                "resumed": [],
                "reason": "missing_reference",
            }
        environment = _number(ambient.get("median_temperature_c"))
        environment_points = _number(ambient.get("point_count"))
        if (
            environment is None
            or environment_points is None
            or environment_points < self.minimum_environment_points
        ):
            return {
                "accepted": [],
                "paused": [],
                "resumed": [],
                "reason": "invalid_reference_quality",
            }

        accepted: list[str] = []
        paused: list[str] = []
        resumed: list[str] = []
        changed = False
        recorded_at = _number(visit.get("recorded_at_unix_sec"))
        for equipment in _mapping_items(visit.get("equipment")):
            equipment_id = str(equipment.get("equipment_id", ""))
            if equipment_id not in self.equipment_ids:
                continue
            entry = self._equipment.setdefault(
                equipment_id,
                {
                    "samples": [],
                    "voxels": {},
                    "paused": False,
                    "stable_streak": 0,
                    "excluded": [],
                    "recent_visit_times": [],
                },
            )
            samples = entry["samples"]
            if not isinstance(samples, list):
                samples = []
                entry["samples"] = samples

            recent_visit_times = [
                value
                for item in entry.get("recent_visit_times", [])
                if (value := _number(item)) is not None
            ]
            recent_limit = self.stability_recovery_visits - 1
            if recorded_at is not None:
                updated_times = (
                    (recent_visit_times + [recorded_at])[-recent_limit:]
                    if recent_limit > 0
                    else []
                )
                if updated_times != recent_visit_times:
                    entry["recent_visit_times"] = updated_times
                    changed = True

            voxels = _mapping_items(equipment.get("voxels"))
            analyses = [
                voxel["trend_analysis"]
                for voxel in voxels
                if isinstance(voxel.get("trend_analysis"), Mapping)
            ]
            critical = any(
                bool(analysis.get("critical"))
                for analysis in analyses
            )
            trend = any(
                bool(analysis.get("trend"))
                for analysis in analyses
            )

            statistics = equipment.get("statistics")
            temperature = (
                _number(statistics.get("p95_temperature_c"))
                if isinstance(statistics, Mapping)
                else None
            )
            sample = None
            if bool(equipment.get("p95_valid")) and temperature is not None:
                sample = {
                    "temperature_c": temperature,
                    "environment_delta_c": temperature - environment,
                }
                if recorded_at is not None:
                    sample["recorded_at_unix_sec"] = recorded_at

            if critical:
                entry["paused"] = True
                entry["stable_streak"] = 0
                self._append_exclusion(
                    entry,
                    "critical_detected",
                    sample=sample,
                    recorded_at=recorded_at,
                )
                paused.append(equipment_id)
                changed = True
                continue

            if trend:
                self._exclude_recent_samples(
                    entry,
                    self.stability_recovery_visits - 1,
                    recent_visit_times,
                )
                entry["paused"] = True
                entry["stable_streak"] = 0
                self._append_exclusion(
                    entry,
                    "trend_detected",
                    sample=sample,
                    recorded_at=recorded_at,
                )
                paused.append(equipment_id)
                changed = True
                continue

            if sample is None:
                continue

            if bool(entry.get("paused", False)):
                stable_streak = int(entry.get("stable_streak", 0)) + 1
                if stable_streak < self.stability_recovery_visits:
                    entry["stable_streak"] = stable_streak
                    self._append_exclusion(
                        entry,
                        "stability_recovery",
                        sample=sample,
                    )
                    changed = True
                    continue
                entry["paused"] = False
                entry["stable_streak"] = 0
                resumed.append(equipment_id)
                changed = True

            if len(samples) >= self.minimum_valid_visits:
                continue

            if recorded_at is not None:
                sample["recorded_at_unix_sec"] = recorded_at
            samples.append(sample)
            voxel_entries = entry["voxels"]
            if not isinstance(voxel_entries, dict):
                voxel_entries = {}
                entry["voxels"] = voxel_entries
            for voxel in voxels:
                voxel_id = str(voxel.get("voxel_id", ""))
                voxel_temperature = _number(
                    voxel.get("p95_temperature_c")
                )
                if not voxel_id or voxel_temperature is None:
                    continue
                voxel_sample = {
                    "temperature_c": voxel_temperature,
                    "environment_delta_c": (
                        voxel_temperature - environment
                    ),
                }
                if recorded_at is not None:
                    voxel_sample["recorded_at_unix_sec"] = recorded_at
                voxel_entries.setdefault(voxel_id, []).append(voxel_sample)
            accepted.append(equipment_id)
            changed = True

        if changed:
            self._save()
        if accepted:
            reason = "recorded"
        elif resumed:
            reason = "collection_resumed"
        elif paused:
            reason = "collection_paused"
        elif any(
            state["paused"] for state in self.recovery_status().values()
        ):
            reason = "stability_recovery"
        else:
            reason = "no_eligible_equipment"
        return {
            "accepted": accepted,
            "paused": paused,
            "resumed": resumed,
            "reason": reason,
            "counts": self.counts(),
            "recovery": self.recovery_status(),
            "ready": self.ready,
        }

    @staticmethod
    def _baseline_stats(
        samples: Sequence[Mapping[str, object]], state: str
    ) -> dict[str, object]:
        temperatures = [float(item["temperature_c"]) for item in samples]
        deltas = [float(item["environment_delta_c"]) for item in samples]
        return {
            "temperature_c": median(temperatures),
            "environment_delta_c": median(deltas),
            "sample_count": len(samples),
            "state": state,
            "operating_state": "normal_load",
        }

    def baseline_document(self, state: str = "validated") -> dict[str, object]:
        if not self.ready:
            raise ValueError("baseline collection is not ready")
        equipment: dict[str, object] = {}
        for equipment_id in self.equipment_ids:
            entry = self._equipment[equipment_id]
            samples = entry["samples"]
            raw_voxels = entry.get("voxels", {})
            voxel_baselines = {
                voxel_id: self._baseline_stats(values, state)
                for voxel_id, values in raw_voxels.items()
                if len(values) >= self.minimum_valid_visits
            }
            equipment[equipment_id] = {
                "equipment": self._baseline_stats(samples, state),
                "voxels": voxel_baselines,
            }
        return {
            "schema_version": 2,
            "note": "Generated automatically from stable patrol visits.",
            "equipment": equipment,
        }

    def approve(self) -> None:
        _atomic_json(
            self.baseline_path,
            self.baseline_document(state="validated"),
        )

    def activate_if_ready(self) -> bool:
        if not self.ready:
            return False
        self.approve()
        return True

    def reset(self) -> None:
        self._equipment.clear()
        if self.collection_path.exists():
            self.collection_path.unlink()
