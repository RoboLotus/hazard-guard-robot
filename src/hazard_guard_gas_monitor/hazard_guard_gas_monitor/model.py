from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class GasVector:
    voc_index: float
    co_ppm: float
    co2_ppm: float

    def add(self, other: "GasVector", scale: float = 1.0) -> "GasVector":
        return GasVector(
            self.voc_index + other.voc_index * scale,
            self.co_ppm + other.co_ppm * scale,
            self.co2_ppm + other.co2_ppm * scale,
        )


@dataclass(frozen=True)
class TimelinePhase:
    phase: str
    start_sec: float
    emission: GasVector
    surface_temperature_c: float


@dataclass(frozen=True)
class GasSource:
    source_id: str
    name: str
    equipment_id: str
    visual_model_id: str
    x: float
    y: float
    dispersion_length_m: float
    wind_x_mps: float
    wind_y_mps: float


@dataclass(frozen=True)
class SensorConfig:
    voc_time_constant_sec: float
    co_time_constant_sec: float
    co2_time_constant_sec: float
    fan_time_constant_multiplier: float
    warmup_sec: float
    noise_seed: int
    voc_noise_std: float
    co_noise_std_ppm: float
    co2_noise_std_ppm: float
    thermal_time_constant_sec: float


@dataclass(frozen=True)
class DecisionConfig:
    voc_watch_delta: float
    voc_absolute_watch_index: float
    voc_persistent_samples: int
    voc_clear_delta: float
    clear_samples: int
    investigation_delay_sec: float
    co_warning_delta_ppm: float
    co_absolute_warning_ppm: float
    co_absolute_critical_ppm: float
    baseline_min_visits: int
    thermal_confirmation_max_age_sec: float


@dataclass(frozen=True)
class GasScenario:
    frame_id: str
    source: GasSource
    ambient: GasVector
    timeline: tuple[TimelinePhase, ...]
    sensor: SensorConfig
    decision: DecisionConfig

    @classmethod
    def load(cls, path: str | Path) -> "GasScenario":
        data: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
        source = data["source"]
        ambient = data["ambient"]
        sensor = data["sensor"]
        decision = data["decision"]
        timeline = tuple(
            TimelinePhase(
                phase=str(item["phase"]),
                start_sec=float(item["start_sec"]),
                emission=GasVector(
                    float(item["voc_delta"]),
                    float(item["co_delta_ppm"]),
                    float(item["co2_delta_ppm"]),
                ),
                surface_temperature_c=float(item["surface_temperature_c"]),
            )
            for item in sorted(data["timeline"], key=lambda value: value["start_sec"])
        )
        if not timeline or timeline[0].start_sec != 0.0:
            raise ValueError("gas timeline must begin at 0 seconds")
        return cls(
            frame_id=str(data.get("frame_id") or "odom"),
            source=GasSource(
                source_id=str(source["id"]),
                name=str(source["name"]),
                equipment_id=str(source["equipment_id"]),
                visual_model_id=str(source.get("visual_model_id") or ""),
                x=float(source["x"]),
                y=float(source["y"]),
                dispersion_length_m=max(0.05, float(source["dispersion_length_m"])),
                wind_x_mps=float(source.get("wind_x_mps", 0.0)),
                wind_y_mps=float(source.get("wind_y_mps", 0.0)),
            ),
            ambient=GasVector(
                float(ambient["voc_index"]),
                float(ambient["co_ppm"]),
                float(ambient["co2_ppm"]),
            ),
            timeline=timeline,
            sensor=SensorConfig(
                **{key: sensor[key] for key in SensorConfig.__annotations__}
            ),
            decision=DecisionConfig(
                **{key: decision[key] for key in DecisionConfig.__annotations__}
            ),
        )

    def phase_at(self, elapsed_sec: float) -> TimelinePhase:
        current = self.timeline[0]
        for candidate in self.timeline[1:]:
            if elapsed_sec < candidate.start_sec:
                break
            current = candidate
        return current

    def concentration_at(
        self,
        x: float,
        y: float,
        elapsed_sec: float,
    ) -> tuple[str, GasVector, float]:
        phase = self.phase_at(elapsed_sec)
        downwind_x = self.source.x + self.source.wind_x_mps * min(elapsed_sec, 20.0)
        downwind_y = self.source.y + self.source.wind_y_mps * min(
            elapsed_sec, 20.0
        )
        distance = math.hypot(x - downwind_x, y - downwind_y)
        strength = math.exp(-distance / self.source.dispersion_length_m)
        return phase.phase, self.ambient.add(phase.emission, strength), distance


class FirstOrderGasSensor:
    """Small, deterministic approximation of sensor lag and noisy readings."""

    def __init__(self, initial: GasVector, config: SensorConfig) -> None:
        self.value = initial
        self.config = config
        self._random = random.Random(int(config.noise_seed))

    @staticmethod
    def _advance(value: float, target: float, dt: float, tau: float) -> float:
        if dt <= 0.0:
            return value
        alpha = 1.0 - math.exp(-dt / max(0.001, tau))
        return value + alpha * (target - value)

    def update(self, target: GasVector, dt: float, fan_on: bool) -> GasVector:
        multiplier = self.config.fan_time_constant_multiplier if fan_on else 1.0
        clean = GasVector(
            self._advance(
                self.value.voc_index,
                target.voc_index,
                dt,
                self.config.voc_time_constant_sec * multiplier,
            ),
            self._advance(
                self.value.co_ppm,
                target.co_ppm,
                dt,
                self.config.co_time_constant_sec * multiplier,
            ),
            self._advance(
                self.value.co2_ppm,
                target.co2_ppm,
                dt,
                self.config.co2_time_constant_sec * multiplier,
            ),
        )
        self.value = clean
        return GasVector(
            max(
                0.0,
                clean.voc_index
                + self._random.gauss(0.0, self.config.voc_noise_std),
            ),
            max(
                0.0,
                clean.co_ppm
                + self._random.gauss(0.0, self.config.co_noise_std_ppm),
            ),
            max(
                0.0,
                clean.co2_ppm
                + self._random.gauss(0.0, self.config.co2_noise_std_ppm),
            ),
        )


class FirstOrderThermalSource:
    """Deterministic surface-temperature lag shared by all simulation views."""

    def __init__(self, initial_temperature_c: float, time_constant_sec: float) -> None:
        self.value_c = float(initial_temperature_c)
        self.time_constant_sec = max(0.001, float(time_constant_sec))

    def update(self, target_temperature_c: float, dt: float) -> float:
        if dt <= 0.0:
            return self.value_c
        alpha = 1.0 - math.exp(-dt / self.time_constant_sec)
        self.value_c += alpha * (float(target_temperature_c) - self.value_c)
        return self.value_c
