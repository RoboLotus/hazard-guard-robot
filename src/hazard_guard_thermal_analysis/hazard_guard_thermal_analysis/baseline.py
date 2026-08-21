"""Approved normal thermal baselines, separate from patrol history."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True)
class BaselineStats:
    temperature_c: float
    sample_count: int
    environment_delta_c: float | None = None
    sigma_normal_c: float = 0.0
    sigma_repeat_c: float = 0.0
    sigma_residual_c: float = 0.0
    sensor_quantization_c: float = 0.0
    state: str = "provisional"
    operating_state: str = "normal_load"

    def validate(self) -> None:
        for name in (
            "temperature_c",
            "sigma_normal_c",
            "sigma_repeat_c",
            "sigma_residual_c",
            "sensor_quantization_c",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or (
                name != "temperature_c" and value < 0.0
            ):
                raise ValueError(
                    f"baseline {name} must be finite and non-negative"
                )
        if (
            self.environment_delta_c is not None
            and not math.isfinite(self.environment_delta_c)
        ):
            raise ValueError("baseline environment_delta_c must be finite")

        if self.sample_count < 8:
            raise ValueError("baseline sample_count must be at least 8")
        if self.state not in {"provisional", "validated"}:
            raise ValueError(
                "baseline state must be provisional or validated"
            )


@dataclass(frozen=True)
class EquipmentBaseline:
    equipment_id: str
    equipment: BaselineStats
    voxels: Mapping[str, BaselineStats]

    def for_voxel(self, voxel_id: str) -> BaselineStats:
        return self.voxels.get(voxel_id, self.equipment)


def _stats(value: object, context: str) -> BaselineStats:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} baseline must be an object")
    result = BaselineStats(
        temperature_c=float(value["temperature_c"]),
        sample_count=int(value["sample_count"]),
        environment_delta_c=(
            float(value["environment_delta_c"])
            if value.get("environment_delta_c") is not None
            else None
        ),
        # Schema 2 deliberately removes sigma from the decision rule. Keep
        # these optional fields so schema-1 files remain readable.
        sigma_normal_c=float(value.get("sigma_normal_c", 0.0)),
        sigma_repeat_c=float(value.get("sigma_repeat_c", 0.0)),
        sigma_residual_c=float(
            value.get("sigma_residual_c", value.get("sigma_normal_c", 0.0))
        ),
        sensor_quantization_c=float(value.get("sensor_quantization_c", 0.0)),
        state=str(value.get("state", "provisional")),
        operating_state=str(
            value.get("operating_state", "normal_load")
        ),
    )
    result.validate()
    return result


def load_baselines(path: str | Path) -> dict[str, EquipmentBaseline]:
    baseline_path = Path(path).expanduser()
    document = json.loads(baseline_path.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("thermal baseline file must be a JSON object")
    schema_version = int(document.get("schema_version", 1))
    if schema_version not in {1, 2}:
        raise ValueError("unsupported thermal baseline schema_version")
    raw_equipment = document.get("equipment", {})
    if not isinstance(raw_equipment, Mapping):
        raise ValueError("thermal baseline equipment must be an object")
    baselines: dict[str, EquipmentBaseline] = {}
    for equipment_id, raw in raw_equipment.items():
        if not isinstance(raw, Mapping):
            raise ValueError(f"baseline for {equipment_id} must be an object")
        raw_voxels = raw.get("voxels", {})
        if not isinstance(raw_voxels, Mapping):
            raise ValueError(
                f"voxel baselines for {equipment_id} must be an object"
            )
        baselines[str(equipment_id)] = EquipmentBaseline(
            equipment_id=str(equipment_id),
            equipment=_stats(
                raw.get("equipment"), f"{equipment_id} equipment"
            ),
            voxels={
                str(voxel_id): _stats(
                    value, f"{equipment_id}/{voxel_id}"
                )
                for voxel_id, value in raw_voxels.items()
            },
        )
    return baselines
