"""Fixed-geometry thermal attribute layer.

The RGB-D mapping pass owns geometry.  This module only associates calibrated
thermal observations with vertices that already exist in an exported PLY and
updates per-vertex statistics.  It deliberately has no ROS dependencies so
the persistence, association and keyframe policy can be tested off-robot.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import struct
import tempfile
import time
from typing import BinaryIO, Iterable

import numpy as np


STATE_SCHEMA_VERSION = 1
_FINGERPRINT_PREFIX = b"hazard-guard-frozen-geometry-v1\0"
_PLY_SCALARS = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


class PlyFormatError(ValueError):
    """Raised when a PLY cannot be loaded without guessing its layout."""


class ThermalStateError(ValueError):
    """Raised when persisted thermal state is unsafe to restore."""


@dataclass(frozen=True)
class _PlyProperty:
    name: str
    scalar_type: str


@dataclass(frozen=True)
class _PlyElement:
    name: str
    count: int
    properties: tuple[_PlyProperty, ...]


@dataclass(frozen=True)
class FixedGeometry:
    """Canonical, voxelised XYZ geometry and its stable fingerprint."""

    points: np.ndarray
    fingerprint: str
    voxel_size_m: float
    source_vertex_count: int

    @classmethod
    def from_points(
        cls,
        points: np.ndarray,
        *,
        voxel_size_m: float = 0.03,
        maximum_voxels: int = 500_000,
    ) -> "FixedGeometry":
        if not math.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be finite and positive")
        if maximum_voxels <= 0:
            raise ValueError("maximum_voxels must be positive")
        source = np.asarray(points, dtype=np.float64)
        if source.ndim != 2 or source.shape[1] != 3:
            raise ValueError("fixed geometry must have shape (N, 3)")
        source_count = int(source.shape[0])
        source = source[np.all(np.isfinite(source), axis=1)]
        if source.shape[0] == 0:
            raise ValueError("fixed geometry has no finite vertices")

        keys = np.floor(source / float(voxel_size_m)).astype(np.int64)
        _, inverse, counts = np.unique(
            keys,
            axis=0,
            return_inverse=True,
            return_counts=True,
        )
        voxel_count = int(counts.shape[0])
        if voxel_count > maximum_voxels:
            raise ValueError(
                "fixed geometry exceeds maximum_voxels after voxelisation: "
                f"{voxel_count} > {maximum_voxels}"
            )
        sums = np.zeros((voxel_count, 3), dtype=np.float64)
        np.add.at(sums, inverse, source)
        canonical = (sums / counts[:, None]).astype("<f4")
        digest = hashlib.sha256()
        digest.update(_FINGERPRINT_PREFIX)
        digest.update(struct.pack("<dQ", float(voxel_size_m), voxel_count))
        digest.update(canonical.tobytes(order="C"))
        return cls(
            points=canonical,
            fingerprint=digest.hexdigest(),
            voxel_size_m=float(voxel_size_m),
            source_vertex_count=source_count,
        )

    @classmethod
    def from_ply(
        cls,
        path: str | Path,
        *,
        voxel_size_m: float = 0.03,
        maximum_voxels: int = 500_000,
        maximum_source_vertices: int = 5_000_000,
    ) -> "FixedGeometry":
        vertices = load_ply_xyz(
            path,
            maximum_source_vertices=maximum_source_vertices,
        )
        return cls.from_points(
            vertices,
            voxel_size_m=voxel_size_m,
            maximum_voxels=maximum_voxels,
        )


def _read_ply_header(
    handle: BinaryIO,
) -> tuple[str, tuple[_PlyElement, ...]]:
    first = handle.readline()
    if first.rstrip(b"\r\n") != b"ply":
        raise PlyFormatError("file does not start with a PLY header")
    ply_format = ""
    elements: list[dict[str, object]] = []
    header_bytes = len(first)
    while True:
        raw = handle.readline()
        if not raw:
            raise PlyFormatError("PLY header is missing end_header")
        header_bytes += len(raw)
        if header_bytes > 1_048_576:
            raise PlyFormatError("PLY header exceeds 1 MiB")
        try:
            line = raw.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise PlyFormatError("PLY header is not ASCII") from exc
        if not line or line.startswith("comment ") or line.startswith("obj_info "):
            continue
        tokens = line.split()
        if tokens[0] == "format":
            if len(tokens) != 3 or tokens[2] != "1.0":
                raise PlyFormatError("only PLY format version 1.0 is supported")
            ply_format = tokens[1]
        elif tokens[0] == "element":
            if len(tokens) != 3:
                raise PlyFormatError("invalid PLY element declaration")
            try:
                count = int(tokens[2])
            except ValueError as exc:
                raise PlyFormatError("invalid PLY element count") from exc
            if count < 0:
                raise PlyFormatError("PLY element count must not be negative")
            elements.append(
                {"name": tokens[1], "count": count, "properties": []}
            )
        elif tokens[0] == "property":
            if not elements:
                raise PlyFormatError("PLY property appears before an element")
            if len(tokens) != 3 or tokens[1] == "list":
                # Faces/cameras can contain lists, but a vertex list would
                # make fixed-width, bounded loading impossible.  Ignore list
                # declarations outside the vertex element because their data
                # follows the vertices and is never read.
                if elements[-1]["name"] == "vertex":
                    raise PlyFormatError("list-valued vertex properties are unsupported")
                continue
            if tokens[1] not in _PLY_SCALARS:
                raise PlyFormatError(
                    f"unsupported PLY scalar type {tokens[1]!r}"
                )
            properties = elements[-1]["properties"]
            assert isinstance(properties, list)
            properties.append(_PlyProperty(tokens[2], tokens[1]))
        elif tokens[0] == "end_header":
            break
    if ply_format not in {
        "ascii",
        "binary_little_endian",
        "binary_big_endian",
    }:
        raise PlyFormatError(f"unsupported PLY format {ply_format!r}")
    parsed = tuple(
        _PlyElement(
            name=str(element["name"]),
            count=int(element["count"]),
            properties=tuple(element["properties"]),
        )
        for element in elements
    )
    return ply_format, parsed


def load_ply_xyz(
    path: str | Path,
    *,
    maximum_source_vertices: int = 5_000_000,
) -> np.ndarray:
    """Load XYZ vertices from PCL/RTAB-Map ASCII or binary PLY output."""
    if maximum_source_vertices <= 0:
        raise ValueError("maximum_source_vertices must be positive")
    source_path = Path(path).expanduser()
    with source_path.open("rb") as handle:
        ply_format, elements = _read_ply_header(handle)
        vertex_position = next(
            (index for index, element in enumerate(elements) if element.name == "vertex"),
            None,
        )
        if vertex_position is None:
            raise PlyFormatError("PLY has no vertex element")
        if any(element.count for element in elements[:vertex_position]):
            raise PlyFormatError("non-empty elements before vertices are unsupported")
        vertex = elements[vertex_position]
        if vertex.count <= 0:
            raise PlyFormatError("PLY vertex element is empty")
        if vertex.count > maximum_source_vertices:
            raise PlyFormatError(
                "PLY vertex count exceeds configured bound: "
                f"{vertex.count} > {maximum_source_vertices}"
            )
        property_names = [prop.name for prop in vertex.properties]
        if any(axis not in property_names for axis in ("x", "y", "z")):
            raise PlyFormatError("PLY vertices require x, y and z properties")

        if ply_format == "ascii":
            xyz_indices = tuple(property_names.index(axis) for axis in ("x", "y", "z"))
            points = np.empty((vertex.count, 3), dtype=np.float64)
            for row in range(vertex.count):
                raw = handle.readline()
                if not raw:
                    raise PlyFormatError("PLY ended before all vertices were read")
                values = np.fromstring(raw.decode("ascii"), sep=" ")
                if values.size < len(vertex.properties):
                    raise PlyFormatError(f"PLY vertex row {row} is too short")
                points[row] = values[list(xyz_indices)]
            return points

        endian = "<" if ply_format == "binary_little_endian" else ">"
        dtype = np.dtype(
            [
                (prop.name, endian + _PLY_SCALARS[prop.scalar_type])
                for prop in vertex.properties
            ]
        )
        records = np.fromfile(handle, dtype=dtype, count=vertex.count)
        if records.shape[0] != vertex.count:
            raise PlyFormatError("PLY ended before all binary vertices were read")
        return np.column_stack(
            [records[axis].astype(np.float64) for axis in ("x", "y", "z")]
        )


class VoxelHashIndex:
    """Bounded-memory spatial lookup over immutable fixed geometry."""

    _NEIGHBOURS = tuple(
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    )

    def __init__(self, geometry: FixedGeometry, cell_size_m: float) -> None:
        if not math.isfinite(cell_size_m) or cell_size_m <= 0.0:
            raise ValueError("cell_size_m must be finite and positive")
        self.geometry = geometry
        self.cell_size_m = float(cell_size_m)
        cells = np.floor(geometry.points / self.cell_size_m).astype(np.int64)
        buckets: dict[tuple[int, int, int], list[int]] = {}
        for index, cell in enumerate(cells):
            key = (int(cell[0]), int(cell[1]), int(cell[2]))
            buckets.setdefault(key, []).append(index)
        self._buckets = {
            key: np.asarray(indices, dtype=np.int32)
            for key, indices in buckets.items()
        }
        self._candidate_cache: dict[
            tuple[int, int, int], np.ndarray
        ] = {}
        self._maximum_candidate_cache_cells = min(
            50_000,
            max(1_024, int(geometry.points.shape[0])),
        )

    def _nearby_candidates(
        self,
        key: tuple[int, int, int],
    ) -> np.ndarray:
        cached = self._candidate_cache.get(key)
        if cached is not None:
            return cached
        cx, cy, cz = key
        neighbours = [
            candidates
            for dx, dy, dz in self._NEIGHBOURS
            if (
                candidates := self._buckets.get(
                    (cx + dx, cy + dy, cz + dz)
                )
            )
            is not None
        ]
        combined = (
            np.concatenate(neighbours)
            if neighbours
            else np.empty(0, dtype=np.int32)
        )
        if (
            combined.shape[0] > 0
            and len(self._candidate_cache)
            < self._maximum_candidate_cache_cells
        ):
            self._candidate_cache[key] = combined
        return combined

    def nearest(
        self,
        points: np.ndarray,
        radius_m: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        observations = np.asarray(points, dtype=np.float32)
        if observations.ndim != 2 or observations.shape[1] != 3:
            raise ValueError("observation points must have shape (N, 3)")
        if radius_m <= 0.0 or radius_m > self.cell_size_m:
            raise ValueError("radius_m must be positive and no larger than cell_size_m")
        indices = np.full(observations.shape[0], -1, dtype=np.int32)
        distances = np.full(observations.shape[0], np.inf, dtype=np.float32)
        radius_squared = float(radius_m) ** 2
        finite = np.all(np.isfinite(observations), axis=1)
        finite_indices = np.flatnonzero(finite)
        if finite_indices.shape[0] == 0:
            return indices, distances
        cells = np.floor(
            observations[finite_indices] / self.cell_size_m
        ).astype(np.int64)
        unique_cells, inverse = np.unique(
            cells,
            axis=0,
            return_inverse=True,
        )
        order = np.argsort(inverse, kind="stable")
        counts = np.bincount(inverse, minlength=unique_cells.shape[0])
        starts = np.r_[0, np.cumsum(counts[:-1])]
        fixed = self.geometry.points
        for group, cell in enumerate(unique_cells):
            local_observations = order[
                starts[group]: starts[group] + counts[group]
            ]
            observation_indices = finite_indices[local_observations]
            key = (int(cell[0]), int(cell[1]), int(cell[2]))
            candidates = self._nearby_candidates(key)
            if candidates.shape[0] == 0:
                continue
            delta = (
                observations[observation_indices, None, :]
                - fixed[candidates][None, :, :]
            )
            squared = np.einsum("ijk,ijk->ij", delta, delta)
            nearest_local = np.argmin(squared, axis=1)
            nearest_squared = squared[
                np.arange(observation_indices.shape[0]),
                nearest_local,
            ]
            within = nearest_squared <= radius_squared
            accepted_observations = observation_indices[within]
            indices[accepted_observations] = candidates[
                nearest_local[within]
            ]
            distances[accepted_observations] = np.sqrt(
                nearest_squared[within]
            )
        return indices, distances


@dataclass(frozen=True)
class AlignmentResult:
    translation: np.ndarray
    candidate_match_ratio: float
    accepted: bool


def estimate_bounded_translation(
    index: VoxelHashIndex,
    observations: np.ndarray,
    *,
    search_radius_m: float,
    maximum_translation_m: float,
    minimum_matches: int = 20,
) -> AlignmentResult:
    """Estimate a conservative local translation without changing any TF.

    This optional hook is intentionally translation-only and disabled by the
    ROS node by default.  A correction outside the configured bound is
    rejected rather than clamped, preventing a poor scene match from snapping
    temperatures onto an unrelated wall.
    """
    points = np.asarray(observations, dtype=np.float32)
    matched, _ = index.nearest(points, search_radius_m)
    mask = matched >= 0
    ratio = float(np.count_nonzero(mask) / max(points.shape[0], 1))
    if np.count_nonzero(mask) < minimum_matches:
        return AlignmentResult(np.zeros(3, dtype=np.float32), ratio, False)
    residuals = index.geometry.points[matched[mask]] - points[mask]
    translation = np.median(residuals, axis=0).astype(np.float32)
    magnitude = float(np.linalg.norm(translation))
    accepted = bool(
        np.all(np.isfinite(translation))
        and magnitude <= maximum_translation_m
    )
    if not accepted:
        translation = np.zeros(3, dtype=np.float32)
    return AlignmentResult(translation, ratio, accepted)


@dataclass(frozen=True)
class IntegrationResult:
    accepted: bool
    valid_observation_count: int
    matched_observation_count: int
    surface_range_rejected_count: int
    updated_voxel_count: int
    match_ratio: float
    alignment_translation: tuple[float, float, float]
    reason: str


class FrozenThermalLayer:
    """Temperature statistics indexed one-for-one by fixed map voxels."""

    def __init__(self, geometry: FixedGeometry, index: VoxelHashIndex) -> None:
        if index.geometry is not geometry:
            raise ValueError("spatial index must reference the layer geometry")
        self.geometry = geometry
        self.index = index
        count = geometry.points.shape[0]
        self.temperature_c = np.full(count, np.nan, dtype=np.float32)
        self.mean_c = np.full(count, np.nan, dtype=np.float32)
        self.minimum_c = np.full(count, np.nan, dtype=np.float32)
        self.maximum_c = np.full(count, np.nan, dtype=np.float32)
        self.m2_c = np.zeros(count, dtype=np.float32)
        self.confidence = np.zeros(count, dtype=np.float32)
        self.observation_count = np.zeros(count, dtype=np.uint32)
        self.last_seen_ns = np.zeros(count, dtype=np.int64)
        self.accepted_frame_count = 0
        self.rejected_frame_count = 0
        self.rejected_observation_count = 0
        self.surface_range_rejected_count = 0
        self.last_match_ratio = 0.0
        self.last_reason = "waiting_for_observations"
        self.last_alignment_translation = (0.0, 0.0, 0.0)
        self.persisted_at_ns = 0
        self.restored = False
        self.dirty = False

    @property
    def observed_mask(self) -> np.ndarray:
        return self.observation_count > 0

    @property
    def observed_voxel_count(self) -> int:
        return int(np.count_nonzero(self.observed_mask))

    @property
    def latest_observation_ns(self) -> int:
        observed = self.observed_mask
        return (
            int(np.max(self.last_seen_ns[observed]))
            if np.any(observed)
            else 0
        )

    def record_rejected_frame(self, observation_count: int, reason: str) -> None:
        self.rejected_frame_count += 1
        self.rejected_observation_count += max(0, int(observation_count))
        self.last_reason = str(reason)

    def integrate(
        self,
        points: np.ndarray,
        temperatures_c: np.ndarray,
        confidences: np.ndarray,
        *,
        observed_at_ns: int,
        association_radius_m: float = 0.08,
        minimum_match_ratio: float = 0.30,
        minimum_observations: int = 20,
        ema_alpha: float = 0.35,
        enable_local_alignment: bool = False,
        alignment_search_radius_m: float = 0.08,
        maximum_alignment_translation_m: float = 0.10,
        sensor_origin: np.ndarray | None = None,
        maximum_surface_range_residual_m: float = 0.05,
    ) -> IntegrationResult:
        observations = np.asarray(points, dtype=np.float32)
        temperatures = np.asarray(temperatures_c, dtype=np.float32).reshape(-1)
        confidence = np.asarray(confidences, dtype=np.float32).reshape(-1)
        if observations.ndim != 2 or observations.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        if temperatures.shape[0] != observations.shape[0] or confidence.shape[0] != observations.shape[0]:
            raise ValueError("points, temperatures and confidences must have equal length")
        if not 0.0 <= minimum_match_ratio <= 1.0:
            raise ValueError("minimum_match_ratio must be in [0, 1]")
        if minimum_observations <= 0:
            raise ValueError("minimum_observations must be positive")
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")

        valid = (
            np.all(np.isfinite(observations), axis=1)
            & np.isfinite(temperatures)
            & np.isfinite(confidence)
            & (confidence > 0.0)
        )
        observations = observations[valid]
        temperatures = temperatures[valid]
        confidence = np.clip(confidence[valid], 0.0, 1.0)
        valid_count = int(observations.shape[0])
        if valid_count < minimum_observations:
            self.record_rejected_frame(valid_count, "insufficient_observations")
            self.last_match_ratio = 0.0
            return IntegrationResult(
                False, valid_count, 0, 0, 0, 0.0, (0.0, 0.0, 0.0),
                "insufficient_observations",
            )

        alignment = np.zeros(3, dtype=np.float32)
        if enable_local_alignment:
            alignment_result = estimate_bounded_translation(
                self.index,
                observations,
                search_radius_m=alignment_search_radius_m,
                maximum_translation_m=maximum_alignment_translation_m,
                minimum_matches=minimum_observations,
            )
            if not alignment_result.accepted:
                self.record_rejected_frame(valid_count, "local_alignment_rejected")
                self.last_match_ratio = alignment_result.candidate_match_ratio
                return IntegrationResult(
                    False,
                    valid_count,
                    0,
                    0,
                    0,
                    alignment_result.candidate_match_ratio,
                    (0.0, 0.0, 0.0),
                    "local_alignment_rejected",
                )
            alignment = alignment_result.translation
            observations = observations + alignment

        matched_indices, distances = self.index.nearest(
            observations,
            association_radius_m,
        )
        surface_range_rejected = 0
        if sensor_origin is not None:
            origin = np.asarray(sensor_origin, dtype=np.float32).reshape(-1)
            if origin.shape != (3,) or not np.all(np.isfinite(origin)):
                raise ValueError("sensor_origin must contain three finite values")
            if maximum_surface_range_residual_m <= 0.0:
                raise ValueError(
                    "maximum_surface_range_residual_m must be positive"
                )
            # Alignment is an internal correction of the complete observation
            # pose, so its sensor origin must move by the same amount.
            origin = origin + alignment
            candidate_mask = matched_indices >= 0
            if np.any(candidate_mask):
                candidate_indices = np.flatnonzero(candidate_mask)
                fixed_ranges = np.linalg.norm(
                    self.geometry.points[matched_indices[candidate_mask]]
                    - origin,
                    axis=1,
                )
                live_ranges = np.linalg.norm(
                    observations[candidate_mask] - origin,
                    axis=1,
                )
                inconsistent = (
                    np.abs(live_ranges - fixed_ranges)
                    > maximum_surface_range_residual_m
                )
                rejected_indices = candidate_indices[inconsistent]
                surface_range_rejected = int(rejected_indices.shape[0])
                matched_indices[rejected_indices] = -1
                distances[rejected_indices] = np.inf
        self.surface_range_rejected_count += surface_range_rejected
        matched_mask = matched_indices >= 0
        matched_count = int(np.count_nonzero(matched_mask))
        match_ratio = float(matched_count / valid_count)
        self.last_match_ratio = match_ratio
        self.last_alignment_translation = tuple(float(value) for value in alignment)
        if match_ratio < minimum_match_ratio:
            self.record_rejected_frame(valid_count, "match_ratio_below_threshold")
            return IntegrationResult(
                False,
                valid_count,
                matched_count,
                surface_range_rejected,
                0,
                match_ratio,
                self.last_alignment_translation,
                "match_ratio_below_threshold",
            )

        targets = matched_indices[matched_mask]
        matched_temperature = temperatures[matched_mask]
        distance_factor = np.maximum(
            0.10,
            1.0 - distances[matched_mask] / float(association_radius_m),
        )
        matched_confidence = confidence[matched_mask] * distance_factor
        order = np.argsort(targets, kind="stable")
        targets = targets[order]
        matched_temperature = matched_temperature[order]
        matched_confidence = matched_confidence[order]
        unique_targets, starts, samples_per_target = np.unique(
            targets,
            return_index=True,
            return_counts=True,
        )
        weights = np.maximum(matched_confidence, 1.0e-6)
        weight_totals = np.add.reduceat(weights, starts)
        frame_temperature = (
            np.add.reduceat(weights * matched_temperature, starts) / weight_totals
        ).astype(np.float32)
        frame_minimum = np.minimum.reduceat(
            matched_temperature,
            starts,
        ).astype(np.float32)
        frame_maximum = np.maximum.reduceat(
            matched_temperature,
            starts,
        ).astype(np.float32)
        frame_confidence = (
            np.add.reduceat(matched_confidence, starts) / samples_per_target
        ).astype(np.float32)

        previous_count = self.observation_count[unique_targets].copy()
        first = previous_count == 0
        if np.any(first):
            first_targets = unique_targets[first]
            first_temperature = frame_temperature[first]
            self.temperature_c[first_targets] = first_temperature
            self.mean_c[first_targets] = first_temperature
            self.minimum_c[first_targets] = frame_minimum[first]
            self.maximum_c[first_targets] = frame_maximum[first]
            self.m2_c[first_targets] = 0.0
            self.confidence[first_targets] = frame_confidence[first]

        if np.any(~first):
            existing_targets = unique_targets[~first]
            sample = frame_temperature[~first]
            old_mean = self.mean_c[existing_targets].copy()
            new_count = previous_count[~first].astype(np.float64) + 1.0
            delta = sample - old_mean
            new_mean = old_mean + delta / new_count
            self.mean_c[existing_targets] = new_mean
            self.m2_c[existing_targets] += delta * (sample - new_mean)
            self.minimum_c[existing_targets] = np.minimum(
                self.minimum_c[existing_targets], frame_minimum[~first]
            )
            self.maximum_c[existing_targets] = np.maximum(
                self.maximum_c[existing_targets], frame_maximum[~first]
            )
            effective_alpha = ema_alpha * (
                0.25 + 0.75 * frame_confidence[~first]
            )
            self.temperature_c[existing_targets] += effective_alpha * (
                sample - self.temperature_c[existing_targets]
            )
            self.confidence[existing_targets] += ema_alpha * (
                frame_confidence[~first] - self.confidence[existing_targets]
            )

        self.observation_count[unique_targets] = np.minimum(
            previous_count.astype(np.uint64) + 1,
            np.iinfo(np.uint32).max,
        ).astype(np.uint32)
        self.last_seen_ns[unique_targets] = max(0, int(observed_at_ns))
        self.accepted_frame_count += 1
        self.rejected_observation_count += valid_count - matched_count
        self.last_reason = "updated"
        self.dirty = True
        return IntegrationResult(
            True,
            valid_count,
            matched_count,
            surface_range_rejected,
            int(unique_targets.shape[0]),
            match_ratio,
            self.last_alignment_translation,
            "updated",
        )

    def save_atomic(self, path: str | Path) -> None:
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        observed = np.flatnonzero(self.observed_mask).astype(np.int32)
        persisted_at_ns = time.time_ns()
        payload = {
            "schema_version": np.asarray(STATE_SCHEMA_VERSION, dtype=np.int32),
            "geometry_fingerprint": np.asarray(self.geometry.fingerprint),
            "geometry_voxel_size_m": np.asarray(
                self.geometry.voxel_size_m, dtype=np.float64
            ),
            "geometry_voxel_count": np.asarray(
                self.geometry.points.shape[0], dtype=np.int64
            ),
            "observed_indices": observed,
            "temperature_c": self.temperature_c[observed],
            "mean_c": self.mean_c[observed],
            "minimum_c": self.minimum_c[observed],
            "maximum_c": self.maximum_c[observed],
            "m2_c": self.m2_c[observed],
            "confidence": self.confidence[observed],
            "observation_count": self.observation_count[observed],
            "last_seen_ns": self.last_seen_ns[observed],
            "accepted_frame_count": np.asarray(
                self.accepted_frame_count, dtype=np.int64
            ),
            "rejected_frame_count": np.asarray(
                self.rejected_frame_count, dtype=np.int64
            ),
            "rejected_observation_count": np.asarray(
                self.rejected_observation_count, dtype=np.int64
            ),
            "surface_range_rejected_count": np.asarray(
                self.surface_range_rejected_count, dtype=np.int64
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
                    "schema_version",
                    "geometry_fingerprint",
                    "geometry_voxel_count",
                    "observed_indices",
                    "temperature_c",
                    "mean_c",
                    "minimum_c",
                    "maximum_c",
                    "m2_c",
                    "confidence",
                    "observation_count",
                    "last_seen_ns",
                }
                missing = required.difference(state.files)
                if missing:
                    raise ThermalStateError(
                        "thermal state is missing fields: "
                        + ", ".join(sorted(missing))
                    )
                schema = int(np.asarray(state["schema_version"]).item())
                if schema != STATE_SCHEMA_VERSION:
                    raise ThermalStateError(
                        f"unsupported thermal state schema {schema}"
                    )
                fingerprint = str(
                    np.asarray(state["geometry_fingerprint"]).item()
                )
                if fingerprint != self.geometry.fingerprint:
                    raise ThermalStateError(
                        "thermal state geometry fingerprint does not match fixed map"
                    )
                geometry_count = int(
                    np.asarray(state["geometry_voxel_count"]).item()
                )
                if geometry_count != self.geometry.points.shape[0]:
                    raise ThermalStateError(
                        "thermal state geometry voxel count does not match fixed map"
                    )
                indices = np.asarray(state["observed_indices"], dtype=np.int64)
                arrays = {
                    name: np.asarray(state[name])
                    for name in (
                        "temperature_c",
                        "mean_c",
                        "minimum_c",
                        "maximum_c",
                        "m2_c",
                        "confidence",
                        "observation_count",
                        "last_seen_ns",
                    )
                }
                if indices.ndim != 1:
                    raise ThermalStateError("observed_indices must be one-dimensional")
                if any(values.shape != indices.shape for values in arrays.values()):
                    raise ThermalStateError("thermal state arrays have inconsistent shapes")
                if (
                    np.any(indices < 0)
                    or np.any(indices >= geometry_count)
                    or np.unique(indices).shape[0] != indices.shape[0]
                ):
                    raise ThermalStateError("thermal state contains invalid voxel indices")
                for name in (
                    "temperature_c",
                    "mean_c",
                    "minimum_c",
                    "maximum_c",
                    "m2_c",
                    "confidence",
                ):
                    if not np.all(np.isfinite(arrays[name])):
                        raise ThermalStateError(
                            f"thermal state {name} contains non-finite values"
                        )
                if np.any(arrays["observation_count"] <= 0):
                    raise ThermalStateError("restored observation counts must be positive")
                if np.any(arrays["last_seen_ns"] < 0):
                    raise ThermalStateError("restored timestamps must not be negative")

                # Validation above is intentionally complete before mutation.
                self.temperature_c[indices] = arrays["temperature_c"]
                self.mean_c[indices] = arrays["mean_c"]
                self.minimum_c[indices] = arrays["minimum_c"]
                self.maximum_c[indices] = arrays["maximum_c"]
                self.m2_c[indices] = arrays["m2_c"]
                self.confidence[indices] = np.clip(
                    arrays["confidence"], 0.0, 1.0
                )
                self.observation_count[indices] = arrays[
                    "observation_count"
                ].astype(np.uint32)
                self.last_seen_ns[indices] = arrays["last_seen_ns"].astype(
                    np.int64
                )
                self.accepted_frame_count = _optional_nonnegative_scalar(
                    state, "accepted_frame_count"
                )
                self.rejected_frame_count = _optional_nonnegative_scalar(
                    state, "rejected_frame_count"
                )
                self.rejected_observation_count = _optional_nonnegative_scalar(
                    state, "rejected_observation_count"
                )
                self.surface_range_rejected_count = _optional_nonnegative_scalar(
                    state, "surface_range_rejected_count"
                )
                self.persisted_at_ns = _optional_nonnegative_scalar(
                    state, "persisted_at_ns"
                )
        except ThermalStateError:
            raise
        except Exception as exc:
            raise ThermalStateError(f"failed to read thermal state: {exc}") from exc
        self.restored = True
        self.dirty = False
        self.last_reason = "restored"


def _optional_nonnegative_scalar(state: object, name: str) -> int:
    files = getattr(state, "files")
    if name not in files:
        return 0
    value = int(np.asarray(state[name]).item())
    if value < 0:
        raise ThermalStateError(f"thermal state {name} must not be negative")
    return value


def quaternion_angular_distance(a: Iterable[float], b: Iterable[float]) -> float:
    quaternion_a = np.asarray(tuple(a), dtype=np.float64)
    quaternion_b = np.asarray(tuple(b), dtype=np.float64)
    if quaternion_a.shape != (4,) or quaternion_b.shape != (4,):
        raise ValueError("quaternions must contain x, y, z, w")
    norm_a = float(np.linalg.norm(quaternion_a))
    norm_b = float(np.linalg.norm(quaternion_b))
    if norm_a <= 1.0e-12 or norm_b <= 1.0e-12:
        raise ValueError("quaternions must not be zero")
    dot = abs(float(np.dot(quaternion_a / norm_a, quaternion_b / norm_b)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def is_keyframe_pose(
    previous: tuple[np.ndarray, np.ndarray] | None,
    translation: np.ndarray,
    quaternion_xyzw: np.ndarray,
    *,
    minimum_translation_m: float = 0.10,
    minimum_rotation_rad: float = math.radians(6.0),
) -> bool:
    if previous is None:
        return True
    previous_translation, previous_quaternion = previous
    translation_delta = float(
        np.linalg.norm(
            np.asarray(translation, dtype=np.float64)
            - np.asarray(previous_translation, dtype=np.float64)
        )
    )
    rotation_delta = quaternion_angular_distance(
        previous_quaternion,
        quaternion_xyzw,
    )
    return bool(
        translation_delta >= minimum_translation_m - 1.0e-9
        or rotation_delta >= minimum_rotation_rad - 1.0e-9
    )


def fixed_stride_indices(total_count: int, maximum_count: int) -> np.ndarray:
    """Choose one immutable, evenly strided visualization subset."""
    if total_count < 0 or maximum_count <= 0:
        raise ValueError("point counts must be non-negative and bounded")
    if total_count <= maximum_count:
        return np.arange(total_count, dtype=np.int32)
    stride = math.ceil(total_count / maximum_count)
    return np.arange(0, total_count, stride, dtype=np.int32)


def thermal_update_due(
    *,
    motion_keyframe: bool,
    seconds_since_success: float,
    seconds_since_rejection: float,
    stationary_refresh_interval_sec: float,
    rejected_frame_retry_sec: float,
) -> bool:
    """Gate expensive matching while still refreshing a stationary surface."""
    if seconds_since_rejection < rejected_frame_retry_sec:
        return False
    return bool(
        motion_keyframe
        or seconds_since_success >= stationary_refresh_interval_sec
    )


class LocalizationStabilityGate:
    """Require consecutive stable localization samples before first update."""

    def __init__(
        self,
        *,
        required_samples: int = 3,
        maximum_translation_m: float = 0.05,
        maximum_rotation_rad: float = math.radians(3.0),
    ) -> None:
        if required_samples <= 0:
            raise ValueError("required_samples must be positive")
        if maximum_translation_m < 0.0 or maximum_rotation_rad < 0.0:
            raise ValueError("localization stability thresholds must be non-negative")
        self.required_samples = int(required_samples)
        self.maximum_translation_m = float(maximum_translation_m)
        self.maximum_rotation_rad = float(maximum_rotation_rad)
        self._anchor: tuple[np.ndarray, np.ndarray] | None = None
        self.sample_count = 0
        self.ready = False

    def observe(
        self,
        translation: np.ndarray,
        quaternion_xyzw: np.ndarray,
    ) -> bool:
        if self.ready:
            return True
        current_translation = np.asarray(translation, dtype=np.float64)
        current_quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
        if current_translation.shape != (3,) or current_quaternion.shape != (4,):
            raise ValueError("localization pose has an invalid shape")
        stable = False
        if self._anchor is not None:
            stable = bool(
                np.linalg.norm(current_translation - self._anchor[0])
                <= self.maximum_translation_m
                and quaternion_angular_distance(
                    current_quaternion,
                    self._anchor[1],
                )
                <= self.maximum_rotation_rad
            )
        if self._anchor is None or not stable:
            self._anchor = (
                current_translation.copy(),
                current_quaternion.copy(),
            )
            self.sample_count = 1
        else:
            self.sample_count += 1
        self.ready = self.sample_count >= self.required_samples
        return self.ready
