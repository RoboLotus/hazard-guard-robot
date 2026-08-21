from __future__ import annotations

import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path

from .model import GasVector


@dataclass(frozen=True)
class GasBaseline:
    voc_index: float
    co_ppm: float
    co2_ppm: float
    visit_count: int

    @property
    def vector(self) -> GasVector:
        return GasVector(self.voc_index, self.co_ppm, self.co2_ppm)


class GasBaselineStore:
    """Persist one median gas vector per confirmed-normal equipment visit."""

    def __init__(self, path: str | Path, min_visits: int) -> None:
        self.path = Path(path).expanduser()
        self.min_visits = max(1, int(min_visits))
        self._records: dict[str, dict[str, object]] = {}
        self._load()

    @staticmethod
    def _key(equipment_id: str, simulated: bool) -> str:
        mode = "simulation" if simulated else "robot"
        return f"{mode}:{equipment_id}"

    @staticmethod
    def _median_vector(readings: list[GasVector]) -> GasVector:
        return GasVector(
            statistics.median(item.voc_index for item in readings),
            statistics.median(item.co_ppm for item in readings),
            statistics.median(item.co2_ppm for item in readings),
        )

    @staticmethod
    def _stored_vector(value: object) -> GasVector | None:
        if not isinstance(value, dict):
            return None
        try:
            vector = GasVector(
                float(value["voc_index"]),
                float(value["co_ppm"]),
                float(value["co2_ppm"]),
            )
        except (KeyError, TypeError, ValueError):
            return None
        if not all(
            math.isfinite(component)
            for component in (vector.voc_index, vector.co_ppm, vector.co2_ppm)
        ):
            return None
        return vector

    @classmethod
    def _valid_samples(cls, record: object) -> list[GasVector]:
        if not isinstance(record, dict) or not isinstance(
            record.get("samples"),
            list,
        ):
            return []
        return [
            vector
            for item in record["samples"]
            if (vector := cls._stored_vector(item)) is not None
        ]

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            records = payload.get("records", {})
            if isinstance(records, dict):
                self._records = records
        except (OSError, TypeError, ValueError):
            self._records = {}

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(f"{self.path.suffix}.tmp")
        temporary.write_text(
            json.dumps(
                {"schema_version": 1, "records": self._records},
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def status(
        self,
        equipment_id: str,
        simulated: bool,
    ) -> tuple[int, GasBaseline | None]:
        record = self._records.get(self._key(equipment_id, simulated), {})
        samples = self._valid_samples(record)
        baseline_data = record.get("baseline") if isinstance(record, dict) else None
        vector = self._stored_vector(baseline_data)
        if vector is None or not isinstance(baseline_data, dict):
            return len(samples), None
        try:
            visit_count = int(baseline_data["visit_count"])
        except (KeyError, TypeError, ValueError):
            return len(samples), None
        if visit_count < self.min_visits:
            return len(samples), None
        return visit_count, GasBaseline(
            vector.voc_index,
            vector.co_ppm,
            vector.co2_ppm,
            visit_count,
        )

    def add_visit(
        self,
        equipment_id: str,
        simulated: bool,
        readings: list[GasVector],
    ) -> GasBaseline | None:
        if not readings:
            return self.status(equipment_id, simulated)[1]
        key = self._key(equipment_id, simulated)
        record = self._records.setdefault(
            key,
            {"samples": [], "baseline": None},
        )
        if isinstance(record, dict) and isinstance(record.get("baseline"), dict):
            _, existing = self.status(equipment_id, simulated)
            if existing is not None:
                return existing
            record["baseline"] = None

        visit = self._median_vector(readings)
        vectors = self._valid_samples(record)
        vectors.append(visit)
        record["samples"] = [asdict(vector) for vector in vectors]

        baseline: GasBaseline | None = None
        if len(vectors) >= self.min_visits:
            median = self._median_vector(vectors[: self.min_visits])
            baseline = GasBaseline(
                median.voc_index,
                median.co_ppm,
                median.co2_ppm,
                self.min_visits,
            )
            record["baseline"] = asdict(baseline)
        self._save()
        return baseline
