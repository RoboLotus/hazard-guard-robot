"""Persistent dynamic voxels layered over immutable map geometry.

The layer consumes calibrated thermal RGB-D points in the global map frame.
It never mutates ``FixedGeometry``.  Points that cannot be explained by the
fixed surface become dynamic candidates, while removal requires positive
line-of-sight evidence from a later depth observation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np

from .frozen_map import FixedGeometry, VoxelHashIndex


DYNAMIC_STATE_SCHEMA_VERSION = 1
_UINT32_MAX = np.iinfo(np.uint32).max
_NEIGHBOURS_26 = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


class DynamicStateError(ValueError):
    """Raised when a dynamic checkpoint is incompatible or malformed."""


@dataclass
class DynamicVoxel:
    key: tuple[int, int, int]
    point: np.ndarray
    temperature_c: float
    mean_c: float
    minimum_c: float
    maximum_c: float
    confidence: float
    observation_count: int
    last_seen_ns: int
    hit_count: int
    miss_count: int
    confirmed: bool


@dataclass(frozen=True)
class DynamicUpdateResult:
    valid_observation_count: int
    static_observation_count: int
    candidate_voxel_count: int
    clustered_candidate_voxel_count: int
    hit_voxel_count: int
    visible_miss_voxel_count: int
    removed_voxel_count: int
    active_voxel_count: int
    confirmed_voxel_count: int


def _saturating_increment(value: int) -> int:
    return min(max(0, int(value)) + 1, int(_UINT32_MAX))


class _RayVisibilityIndex:
    """Sparse angular depth buffer used to distinguish unseen from absent."""

    def __init__(
        self,
        points: np.ndarray,
        sensor_origin: np.ndarray,
        angular_resolution_rad: float,
    ) -> None:
        if not math.isfinite(angular_resolution_rad) or angular_resolution_rad <= 0:
            raise ValueError("angular_resolution_rad must be finite and positive")
        self.origin = np.asarray(sensor_origin, dtype=np.float32).reshape(3)
        self.resolution = float(angular_resolution_rad)
        self._azimuth_bins = max(1, math.ceil(2.0 * math.pi / self.resolution))
        self._azimuth_resolution = 2.0 * math.pi / self._azimuth_bins
        vectors = np.asarray(points, dtype=np.float32) - self.origin
        ranges = np.linalg.norm(vectors, axis=1)
        valid = np.all(np.isfinite(vectors), axis=1) & (ranges > 1.0e-4)
        vectors = vectors[valid]
        ranges = ranges[valid]
        self._minimum_ranges: dict[tuple[int, int], float] = {}
        if ranges.shape[0] == 0:
            return
        keys = self._keys(vectors)
        for key, depth in zip(keys, ranges):
            item = (int(key[0]), int(key[1]))
            previous = self._minimum_ranges.get(item)
            if previous is None or float(depth) < previous:
                self._minimum_ranges[item] = float(depth)

    def _keys(self, vectors: np.ndarray) -> np.ndarray:
        horizontal = np.hypot(vectors[:, 0], vectors[:, 1])
        azimuth = np.arctan2(vectors[:, 1], vectors[:, 0])
        elevation = np.arctan2(vectors[:, 2], horizontal)
        azimuth_key = np.floor(
            (azimuth + math.pi) / self._azimuth_resolution
        ).astype(np.int32) % self._azimuth_bins
        elevation_key = np.floor(elevation / self.resolution).astype(np.int32)
        return np.column_stack((azimuth_key, elevation_key))

    def was_observed(
        self,
        point: np.ndarray,
        *,
        range_tolerance_m: float,
    ) -> bool:
        vector = np.asarray(point, dtype=np.float32).reshape(3) - self.origin
        target_range = float(np.linalg.norm(vector))
        if not math.isfinite(target_range) or target_range <= 1.0e-4:
            return False
        key = self._keys(vector.reshape(1, 3))[0]
        observed_ranges = [
            depth
            for da in (-1, 0, 1)
            for de in (-1, 0, 1)
            if (
                depth := self._minimum_ranges.get(
                    (
                        (int(key[0]) + da) % self._azimuth_bins,
                        int(key[1]) + de,
                    )
                )
            )
            is not None
        ]
        if not observed_ranges:
            return False
        # A nearer return means the old voxel is occluded, not absent.  A
        # return at or behind its previous range positively re-observes it.
        return min(observed_ranges) >= target_range - range_tolerance_m


def _connected_large_keys(
    keys: set[tuple[int, int, int]],
    minimum_component_voxels: int,
) -> set[tuple[int, int, int]]:
    remaining = set(keys)
    accepted: set[tuple[int, int, int]] = set()
    while remaining:
        seed = remaining.pop()
        component = {seed}
        stack = [seed]
        while stack:
            x, y, z = stack.pop()
            for dx, dy, dz in _NEIGHBOURS_26:
                neighbour = (x + dx, y + dy, z + dz)
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    component.add(neighbour)
                    stack.append(neighbour)
        if len(component) >= minimum_component_voxels:
            accepted.update(component)
    return accepted


class DynamicVoxelLayer:
    """Voxel-level persistent geometry and temperature state."""

    def __init__(
        self,
        geometry: FixedGeometry,
        static_index: VoxelHashIndex,
        *,
        voxel_size_m: float = 0.05,
        static_association_radius_m: float = 0.08,
        maximum_static_range_residual_m: float = 0.05,
        minimum_component_voxels: int = 8,
        minimum_hits: int = 2,
        maximum_misses: int = 3,
        maximum_voxels: int = 50_000,
        visibility_angular_resolution_deg: float = 1.0,
        visibility_range_tolerance_m: float = 0.08,
        temperature_ema_alpha: float = 0.35,
    ) -> None:
        if static_index.geometry is not geometry:
            raise ValueError("static index must reference immutable geometry")
        if voxel_size_m <= 0 or not math.isfinite(voxel_size_m):
            raise ValueError("voxel_size_m must be finite and positive")
        if not 0 < static_association_radius_m <= static_index.cell_size_m:
            raise ValueError(
                "static_association_radius_m must be positive and no larger "
                "than the static index cell size"
            )
        if maximum_static_range_residual_m <= 0:
            raise ValueError("maximum_static_range_residual_m must be positive")
        if minimum_component_voxels <= 0 or minimum_hits <= 0:
            raise ValueError("component and hit thresholds must be positive")
        if maximum_misses <= 0 or maximum_voxels <= 0:
            raise ValueError("miss and voxel limits must be positive")
        if visibility_angular_resolution_deg <= 0:
            raise ValueError("visibility angular resolution must be positive")
        if visibility_range_tolerance_m < 0:
            raise ValueError("visibility range tolerance must not be negative")
        if not 0.0 < temperature_ema_alpha <= 1.0:
            raise ValueError("temperature_ema_alpha must be in (0, 1]")
        self.geometry = geometry
        self.static_index = static_index
        self.voxel_size_m = float(voxel_size_m)
        self.static_association_radius_m = float(static_association_radius_m)
        self.maximum_static_range_residual_m = float(
            maximum_static_range_residual_m
        )
        self.minimum_component_voxels = int(minimum_component_voxels)
        self.minimum_hits = int(minimum_hits)
        self.maximum_misses = int(maximum_misses)
        self.maximum_voxels = int(maximum_voxels)
        self.visibility_angular_resolution_rad = math.radians(
            float(visibility_angular_resolution_deg)
        )
        self.visibility_range_tolerance_m = float(
            visibility_range_tolerance_m
        )
        self.temperature_ema_alpha = float(temperature_ema_alpha)
        self._voxels: dict[tuple[int, int, int], DynamicVoxel] = {}
        self.persisted_at_ns = 0
        self.restored = False
        self.dirty = False

    @property
    def active_voxel_count(self) -> int:
        return len(self._voxels)

    @property
    def confirmed_voxel_count(self) -> int:
        return sum(voxel.confirmed for voxel in self._voxels.values())

    @property
    def latest_observation_ns(self) -> int:
        return max(
            (voxel.last_seen_ns for voxel in self._voxels.values()),
            default=0,
        )

    def _voxelise(
        self,
        points: np.ndarray,
        temperatures: np.ndarray,
        confidences: np.ndarray,
    ) -> dict[tuple[int, int, int], tuple[np.ndarray, float, float, float, float]]:
        keys = np.floor(points / self.voxel_size_m).astype(np.int64)
        unique, inverse, counts = np.unique(
            keys, axis=0, return_inverse=True, return_counts=True
        )
        sums = np.zeros((unique.shape[0], 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        centroids = (sums / counts[:, None]).astype(np.float32)
        result: dict[
            tuple[int, int, int], tuple[np.ndarray, float, float, float, float]
        ] = {}
        for index, raw_key in enumerate(unique):
            mask = inverse == index
            valid_temperature = (
                np.isfinite(temperatures[mask])
                & np.isfinite(confidences[mask])
                & (confidences[mask] > 0.0)
            )
            local_temperatures = temperatures[mask][valid_temperature]
            local_confidence = np.clip(
                confidences[mask][valid_temperature], 0.0, 1.0
            )
            if local_temperatures.shape[0]:
                weights = np.maximum(local_confidence, 1.0e-6)
                temperature = float(
                    np.sum(local_temperatures * weights) / np.sum(weights)
                )
                minimum = float(np.min(local_temperatures))
                maximum = float(np.max(local_temperatures))
                confidence = float(np.mean(local_confidence))
            else:
                temperature = math.nan
                minimum = math.nan
                maximum = math.nan
                confidence = 0.0
            key = tuple(int(value) for value in raw_key)
            result[key] = (
                centroids[index], temperature, minimum, maximum, confidence
            )
        return result

    def _static_mask(
        self,
        points: np.ndarray,
        sensor_origin: np.ndarray,
    ) -> np.ndarray:
        matched, _ = self.static_index.nearest(
            points, self.static_association_radius_m
        )
        static = matched >= 0
        if np.any(static):
            indices = np.flatnonzero(static)
            fixed_ranges = np.linalg.norm(
                self.geometry.points[matched[static]] - sensor_origin,
                axis=1,
            )
            live_ranges = np.linalg.norm(points[static] - sensor_origin, axis=1)
            inconsistent = (
                np.abs(live_ranges - fixed_ranges)
                > self.maximum_static_range_residual_m
            )
            static[indices[inconsistent]] = False
        return static

    def integrate(
        self,
        points: np.ndarray,
        temperatures_c: np.ndarray,
        confidences: np.ndarray,
        *,
        observed_at_ns: int,
        sensor_origin: np.ndarray,
    ) -> DynamicUpdateResult:
        observations = np.asarray(points, dtype=np.float32)
        temperatures = np.asarray(temperatures_c, dtype=np.float32).reshape(-1)
        confidence = np.asarray(confidences, dtype=np.float32).reshape(-1)
        origin = np.asarray(sensor_origin, dtype=np.float32).reshape(-1)
        if observations.ndim != 2 or observations.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        if (
            temperatures.shape[0] != observations.shape[0]
            or confidence.shape[0] != observations.shape[0]
        ):
            raise ValueError("points, temperatures and confidences must match")
        if origin.shape != (3,) or not np.all(np.isfinite(origin)):
            raise ValueError("sensor_origin must contain three finite values")
        valid = np.all(np.isfinite(observations), axis=1)
        observations = observations[valid]
        temperatures = temperatures[valid]
        confidence = confidence[valid]
        valid_count = int(observations.shape[0])
        if valid_count == 0:
            return DynamicUpdateResult(
                0, 0, 0, 0, 0, 0, 0,
                self.active_voxel_count, self.confirmed_voxel_count,
            )

        static_mask = self._static_mask(observations, origin)
        dynamic_points = observations[~static_mask]
        dynamic_temperatures = temperatures[~static_mask]
        dynamic_confidence = confidence[~static_mask]
        candidates = self._voxelise(
            dynamic_points, dynamic_temperatures, dynamic_confidence
        ) if dynamic_points.shape[0] else {}
        candidate_keys = set(candidates)
        clustered_keys = _connected_large_keys(
            candidate_keys, self.minimum_component_voxels
        )
        # Once a candidate exists, a partial view may update it without having
        # to satisfy the new-object size gate again.
        update_keys = clustered_keys | (candidate_keys & self._voxels.keys())
        observed_keys: set[tuple[int, int, int]] = set()
        timestamp = max(0, int(observed_at_ns))
        for key in update_keys:
            point, temperature, minimum, maximum, frame_confidence = candidates[key]
            voxel = self._voxels.get(key)
            if voxel is None:
                voxel = DynamicVoxel(
                    key=key,
                    point=point.copy(),
                    temperature_c=temperature,
                    mean_c=temperature,
                    minimum_c=minimum,
                    maximum_c=maximum,
                    confidence=frame_confidence,
                    observation_count=1 if math.isfinite(temperature) else 0,
                    last_seen_ns=timestamp,
                    hit_count=1,
                    miss_count=0,
                    confirmed=self.minimum_hits <= 1,
                )
                self._voxels[key] = voxel
            else:
                voxel.point += 0.35 * (point - voxel.point)
                voxel.hit_count = _saturating_increment(voxel.hit_count)
                voxel.miss_count = 0
                voxel.last_seen_ns = timestamp
                if math.isfinite(temperature):
                    if voxel.observation_count <= 0 or not math.isfinite(
                        voxel.temperature_c
                    ):
                        voxel.temperature_c = temperature
                        voxel.mean_c = temperature
                        voxel.minimum_c = minimum
                        voxel.maximum_c = maximum
                        voxel.confidence = frame_confidence
                        voxel.observation_count = 1
                    else:
                        count = voxel.observation_count + 1
                        voxel.mean_c += (temperature - voxel.mean_c) / count
                        voxel.minimum_c = min(voxel.minimum_c, minimum)
                        voxel.maximum_c = max(voxel.maximum_c, maximum)
                        alpha = self.temperature_ema_alpha * (
                            0.25 + 0.75 * frame_confidence
                        )
                        voxel.temperature_c += alpha * (
                            temperature - voxel.temperature_c
                        )
                        voxel.confidence += self.temperature_ema_alpha * (
                            frame_confidence - voxel.confidence
                        )
                        voxel.observation_count = min(count, int(_UINT32_MAX))
                if voxel.hit_count >= self.minimum_hits:
                    voxel.confirmed = True
            observed_keys.add(key)

        visibility = _RayVisibilityIndex(
            observations, origin, self.visibility_angular_resolution_rad
        )
        visible_misses = 0
        removed = 0
        for key, voxel in list(self._voxels.items()):
            if key in observed_keys:
                continue
            if visibility.was_observed(
                voxel.point,
                range_tolerance_m=self.visibility_range_tolerance_m,
            ):
                voxel.miss_count = _saturating_increment(voxel.miss_count)
                visible_misses += 1
                if voxel.miss_count >= self.maximum_misses:
                    del self._voxels[key]
                    removed += 1

        if len(self._voxels) > self.maximum_voxels:
            overflow = len(self._voxels) - self.maximum_voxels
            eviction = sorted(
                self._voxels.values(),
                key=lambda voxel: (
                    voxel.confirmed,
                    voxel.hit_count,
                    voxel.last_seen_ns,
                ),
            )[:overflow]
            for voxel in eviction:
                del self._voxels[voxel.key]
                removed += 1
        if update_keys or visible_misses or removed:
            self.dirty = True
        return DynamicUpdateResult(
            valid_observation_count=valid_count,
            static_observation_count=int(np.count_nonzero(static_mask)),
            candidate_voxel_count=len(candidate_keys),
            clustered_candidate_voxel_count=len(clustered_keys),
            hit_voxel_count=len(observed_keys),
            visible_miss_voxel_count=visible_misses,
            removed_voxel_count=removed,
            active_voxel_count=self.active_voxel_count,
            confirmed_voxel_count=self.confirmed_voxel_count,
        )

    def snapshot(
        self,
        *,
        confirmed_only: bool = True,
        maximum_voxels: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        voxels = [
            voxel
            for voxel in self._voxels.values()
            if (voxel.confirmed or not confirmed_only)
            and math.isfinite(voxel.temperature_c)
        ]
        voxels.sort(key=lambda voxel: voxel.key)
        if maximum_voxels is not None and len(voxels) > maximum_voxels:
            step = max(1, math.ceil(len(voxels) / maximum_voxels))
            voxels = voxels[::step][:maximum_voxels]
        if not voxels:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty(0, dtype=np.float32),
                np.empty(0, dtype=np.float32),
                np.empty(0, dtype=np.uint32),
                np.empty(0, dtype=np.uint32),
                np.empty(0, dtype=np.int64),
            )
        return (
            np.asarray([voxel.point for voxel in voxels], dtype=np.float32),
            np.asarray([voxel.temperature_c for voxel in voxels], dtype=np.float32),
            np.asarray([voxel.confidence for voxel in voxels], dtype=np.float32),
            np.asarray([voxel.hit_count for voxel in voxels], dtype=np.uint32),
            np.asarray([voxel.miss_count for voxel in voxels], dtype=np.uint32),
            np.asarray([voxel.last_seen_ns for voxel in voxels], dtype=np.int64),
        )

    def save_atomic(self, path: str | Path) -> None:
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        voxels = sorted(self._voxels.values(), key=lambda voxel: voxel.key)
        count = len(voxels)
        persisted_at_ns = time.time_ns()
        payload = {
            "schema_version": np.asarray(
                DYNAMIC_STATE_SCHEMA_VERSION, dtype=np.int32
            ),
            "geometry_fingerprint": np.asarray(self.geometry.fingerprint),
            "dynamic_voxel_size_m": np.asarray(
                self.voxel_size_m, dtype=np.float64
            ),
            "keys": np.asarray(
                [voxel.key for voxel in voxels], dtype=np.int64
            ).reshape(count, 3),
            "points": np.asarray(
                [voxel.point for voxel in voxels], dtype=np.float32
            ).reshape(count, 3),
            "temperature_c": np.asarray(
                [voxel.temperature_c for voxel in voxels], dtype=np.float32
            ),
            "mean_c": np.asarray(
                [voxel.mean_c for voxel in voxels], dtype=np.float32
            ),
            "minimum_c": np.asarray(
                [voxel.minimum_c for voxel in voxels], dtype=np.float32
            ),
            "maximum_c": np.asarray(
                [voxel.maximum_c for voxel in voxels], dtype=np.float32
            ),
            "confidence": np.asarray(
                [voxel.confidence for voxel in voxels], dtype=np.float32
            ),
            "observation_count": np.asarray(
                [voxel.observation_count for voxel in voxels], dtype=np.uint32
            ),
            "last_seen_ns": np.asarray(
                [voxel.last_seen_ns for voxel in voxels], dtype=np.int64
            ),
            "hit_count": np.asarray(
                [voxel.hit_count for voxel in voxels], dtype=np.uint32
            ),
            "miss_count": np.asarray(
                [voxel.miss_count for voxel in voxels], dtype=np.uint32
            ),
            "confirmed": np.asarray(
                [voxel.confirmed for voxel in voxels], dtype=np.bool_
            ),
            "persisted_at_ns": np.asarray(persisted_at_ns, dtype=np.int64),
        }
        temporary_name = ""
        try:
            with tempfile.NamedTemporaryFile(
                mode="w+b",
                prefix=f".{destination.name}.",
                suffix=".tmp",
                dir=destination.parent,
                delete=False,
            ) as temporary:
                temporary_name = temporary.name
                np.savez_compressed(temporary, **payload)
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_name, destination)
            directory_fd = os.open(destination.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except Exception:
            if temporary_name:
                Path(temporary_name).unlink(missing_ok=True)
            raise
        self.persisted_at_ns = persisted_at_ns
        self.dirty = False

    def restore(self, path: str | Path) -> None:
        source = Path(path).expanduser()
        try:
            with np.load(source, allow_pickle=False) as state:
                required = {
                    "schema_version", "geometry_fingerprint",
                    "dynamic_voxel_size_m", "keys", "points",
                    "temperature_c", "mean_c", "minimum_c", "maximum_c",
                    "confidence", "observation_count", "last_seen_ns",
                    "hit_count", "miss_count", "confirmed",
                }
                missing = required.difference(state.files)
                if missing:
                    raise DynamicStateError(
                        "dynamic state is missing fields: "
                        + ", ".join(sorted(missing))
                    )
                schema = int(np.asarray(state["schema_version"]).item())
                if schema != DYNAMIC_STATE_SCHEMA_VERSION:
                    raise DynamicStateError(
                        f"unsupported dynamic state schema {schema}"
                    )
                fingerprint = str(
                    np.asarray(state["geometry_fingerprint"]).item()
                )
                if fingerprint != self.geometry.fingerprint:
                    raise DynamicStateError(
                        "dynamic state geometry fingerprint does not match fixed map"
                    )
                voxel_size = float(
                    np.asarray(state["dynamic_voxel_size_m"]).item()
                )
                if not math.isclose(voxel_size, self.voxel_size_m, abs_tol=1e-9):
                    raise DynamicStateError(
                        "dynamic state voxel size does not match configuration"
                    )
                keys = np.asarray(state["keys"], dtype=np.int64)
                points = np.asarray(state["points"], dtype=np.float32)
                if keys.ndim != 2 or keys.shape[1:] != (3,):
                    raise DynamicStateError("dynamic keys must have shape (N, 3)")
                if points.shape != keys.shape:
                    raise DynamicStateError("dynamic points must match key shape")
                count = keys.shape[0]
                arrays = {
                    name: np.asarray(state[name])
                    for name in (
                        "temperature_c", "mean_c", "minimum_c", "maximum_c",
                        "confidence", "observation_count", "last_seen_ns",
                        "hit_count", "miss_count", "confirmed",
                    )
                }
                if any(values.shape != (count,) for values in arrays.values()):
                    raise DynamicStateError(
                        "dynamic state arrays have inconsistent shapes"
                    )
                if np.unique(keys, axis=0).shape[0] != count:
                    raise DynamicStateError("dynamic state contains duplicate keys")
                if count > self.maximum_voxels:
                    raise DynamicStateError(
                        "dynamic state exceeds configured voxel limit"
                    )
                if not np.all(np.isfinite(points)):
                    raise DynamicStateError("dynamic state points are non-finite")
                temperature_arrays = (
                    arrays["temperature_c"], arrays["mean_c"],
                    arrays["minimum_c"], arrays["maximum_c"],
                )
                finite_pattern = np.isfinite(temperature_arrays[0])
                if any(
                    not np.array_equal(np.isfinite(values), finite_pattern)
                    for values in temperature_arrays[1:]
                ):
                    raise DynamicStateError(
                        "dynamic temperature statistics are inconsistent"
                    )
                if np.any(arrays["last_seen_ns"] < 0):
                    raise DynamicStateError("dynamic timestamps must not be negative")
                if not np.all(np.isfinite(arrays["confidence"])):
                    raise DynamicStateError(
                        "dynamic confidence contains non-finite values"
                    )
                if np.any(arrays["hit_count"] <= 0):
                    raise DynamicStateError("dynamic hit counts must be positive")
                if np.any(arrays["miss_count"] >= self.maximum_misses):
                    raise DynamicStateError(
                        "dynamic state contains voxels past the miss limit"
                    )
                has_temperature = arrays["observation_count"] > 0
                if not np.array_equal(has_temperature, finite_pattern):
                    raise DynamicStateError(
                        "dynamic observation counts do not match temperatures"
                    )
                confirmed = arrays["confirmed"].astype(np.bool_)
                if np.any(
                    confirmed
                    & (arrays["hit_count"] < self.minimum_hits)
                ):
                    raise DynamicStateError(
                        "confirmed dynamic voxels are below the hit threshold"
                    )

                restored: dict[tuple[int, int, int], DynamicVoxel] = {}
                for index in range(count):
                    key = tuple(int(value) for value in keys[index])
                    restored[key] = DynamicVoxel(
                        key=key,
                        point=points[index].copy(),
                        temperature_c=float(arrays["temperature_c"][index]),
                        mean_c=float(arrays["mean_c"][index]),
                        minimum_c=float(arrays["minimum_c"][index]),
                        maximum_c=float(arrays["maximum_c"][index]),
                        confidence=float(np.clip(arrays["confidence"][index], 0, 1)),
                        observation_count=int(arrays["observation_count"][index]),
                        last_seen_ns=int(arrays["last_seen_ns"][index]),
                        hit_count=int(arrays["hit_count"][index]),
                        miss_count=int(arrays["miss_count"][index]),
                        confirmed=bool(arrays["confirmed"][index]),
                    )
                self._voxels = restored
                self.persisted_at_ns = (
                    int(np.asarray(state["persisted_at_ns"]).item())
                    if "persisted_at_ns" in state.files
                    else 0
                )
        except DynamicStateError:
            raise
        except Exception as exc:
            raise DynamicStateError(
                f"failed to restore dynamic state: {exc}"
            ) from exc
        self.restored = True
        self.dirty = False
