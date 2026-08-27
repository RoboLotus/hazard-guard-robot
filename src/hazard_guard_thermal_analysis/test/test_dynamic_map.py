from pathlib import Path

import numpy as np
import pytest

from hazard_guard_thermal_analysis.dynamic_map import (
    DynamicStateError,
    DynamicVoxelLayer,
)
from hazard_guard_thermal_analysis.frozen_map import FixedGeometry, VoxelHashIndex


def _layer(
    *,
    minimum_hits: int = 2,
    maximum_misses: int = 2,
) -> DynamicVoxelLayer:
    static_points = np.asarray(
        [[2.0, 0.0, 0.0], [2.0, 0.1, 0.0], [2.0, 0.2, 0.0]],
        dtype=np.float32,
    )
    geometry = FixedGeometry.from_points(
        static_points, voxel_size_m=0.02, maximum_voxels=20
    )
    index = VoxelHashIndex(geometry, cell_size_m=0.12)
    return DynamicVoxelLayer(
        geometry,
        index,
        voxel_size_m=0.05,
        static_association_radius_m=0.12,
        maximum_static_range_residual_m=0.04,
        minimum_component_voxels=3,
        minimum_hits=minimum_hits,
        maximum_misses=maximum_misses,
        maximum_voxels=100,
        visibility_angular_resolution_deg=2.0,
        visibility_range_tolerance_m=0.03,
    )


def _observe(
    layer: DynamicVoxelLayer,
    points: np.ndarray,
    stamp: int,
):
    count = points.shape[0]
    return layer.integrate(
        points,
        np.linspace(35.0, 45.0, count, dtype=np.float32),
        np.ones(count, dtype=np.float32),
        observed_at_ns=stamp,
        sensor_origin=np.zeros(3, dtype=np.float32),
    )


def test_connected_new_geometry_requires_hits_and_does_not_mutate_static_map() -> None:
    layer = _layer()
    original = layer.geometry.points.copy()
    dynamic = np.asarray(
        [[1.0, 0.00, 0.0], [1.0, 0.05, 0.0], [1.0, 0.10, 0.0]],
        dtype=np.float32,
    )

    first = _observe(layer, dynamic, 10)
    assert first.clustered_candidate_voxel_count == 3
    assert set(first.created_keys) == set(layer._voxels)
    assert first.updated_keys == ()
    assert first.deleted_keys == ()
    assert layer.active_voxel_count == 3
    assert layer.confirmed_voxel_count == 0

    second = _observe(layer, dynamic, 20)
    assert second.hit_voxel_count == 3
    assert second.created_keys == ()
    assert set(second.updated_keys) == set(layer._voxels)
    assert second.deleted_keys == ()
    assert layer.confirmed_voxel_count == 3
    points, temperatures, _, hits, misses, last_seen = layer.snapshot()
    assert points.shape == (3, 3)
    assert np.all(np.isfinite(temperatures))
    assert hits.tolist() == [2, 2, 2]
    assert misses.tolist() == [0, 0, 0]
    assert last_seen.tolist() == [20, 20, 20]
    np.testing.assert_array_equal(layer.geometry.points, original)


def test_small_one_frame_noise_is_never_created() -> None:
    layer = _layer(minimum_hits=1)
    result = _observe(
        layer,
        np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
        10,
    )
    assert result.candidate_voxel_count == 1
    assert result.clustered_candidate_voxel_count == 0
    assert layer.active_voxel_count == 0


def test_out_of_fov_and_occluded_voxels_do_not_receive_misses() -> None:
    layer = _layer(minimum_hits=1)
    dynamic = np.asarray(
        [[1.0, 0.00, 0.0], [1.0, 0.05, 0.0], [1.0, 0.10, 0.0]],
        dtype=np.float32,
    )
    _observe(layer, dynamic, 10)

    out_of_fov = np.asarray(
        [[0.0, 1.0, 0.0], [0.0, 1.05, 0.0], [0.0, 1.10, 0.0]],
        dtype=np.float32,
    )
    _observe(layer, out_of_fov, 20)
    assert layer.snapshot()[4].tolist() == [0, 0, 0]

    occluder = dynamic * 0.5
    _observe(layer, occluder, 30)
    original_misses = [
        voxel.miss_count
        for key, voxel in layer._voxels.items()
        if key[0] >= 20
    ]
    assert original_misses == [0, 0, 0]


def test_positive_background_observation_removes_dynamic_voxels_after_misses() -> None:
    layer = _layer(minimum_hits=1, maximum_misses=2)
    dynamic = np.asarray(
        [[1.0, 0.00, 0.0], [1.0, 0.05, 0.0], [1.0, 0.10, 0.0]],
        dtype=np.float32,
    )
    background = np.asarray(
        [[2.0, 0.00, 0.0], [2.0, 0.10, 0.0], [2.0, 0.20, 0.0]],
        dtype=np.float32,
    )
    _observe(layer, dynamic, 10)
    first_miss = _observe(layer, background, 20)
    assert first_miss.visible_miss_voxel_count == 3
    assert first_miss.removed_voxel_count == 0
    assert layer.confirmed_voxel_count == 3

    second_miss = _observe(layer, background, 30)
    assert second_miss.removed_voxel_count == 3
    assert second_miss.created_keys == ()
    assert second_miss.updated_keys == ()
    assert len(second_miss.deleted_keys) == 3
    assert layer.active_voxel_count == 0
    # Removal affects only the dynamic dictionary; the static wall remains.
    assert layer.geometry.points.shape == (3, 3)


def test_dynamic_checkpoint_round_trip_and_fingerprint_gate(tmp_path: Path) -> None:
    layer = _layer(minimum_hits=1)
    dynamic = np.asarray(
        [[1.0, 0.00, 0.0], [1.0, 0.05, 0.0], [1.0, 0.10, 0.0]],
        dtype=np.float32,
    )
    _observe(layer, dynamic, 123)
    state = tmp_path / "dynamic_layer.npz"
    layer.save_atomic(state)

    restored = _layer(minimum_hits=1)
    restored.restore(state)
    assert restored.restored
    assert restored.confirmed_voxel_count == 3
    assert restored.snapshot()[5].tolist() == [123, 123, 123]

    other_geometry = FixedGeometry.from_points(
        np.asarray([[9.0, 0.0, 0.0]], dtype=np.float32),
        voxel_size_m=0.02,
        maximum_voxels=10,
    )
    incompatible = DynamicVoxelLayer(
        other_geometry,
        VoxelHashIndex(other_geometry, cell_size_m=0.12),
        voxel_size_m=0.05,
        static_association_radius_m=0.12,
        minimum_component_voxels=1,
    )
    with pytest.raises(DynamicStateError, match="fingerprint"):
        incompatible.restore(state)
