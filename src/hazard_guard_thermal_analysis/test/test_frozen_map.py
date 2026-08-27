import math
from pathlib import Path
import struct

import numpy as np
import pytest

from hazard_guard_thermal_analysis.frozen_map import (
    FixedGeometry,
    FrozenThermalLayer,
    LocalizationStabilityGate,
    PlyFormatError,
    ThermalStateError,
    VoxelHashIndex,
    estimate_bounded_translation,
    fixed_stride_indices,
    is_keyframe_pose,
    load_ply_xyz,
    thermal_update_due,
)


def _layer(points: list[tuple[float, float, float]]) -> FrozenThermalLayer:
    geometry = FixedGeometry.from_points(
        np.asarray(points, dtype=np.float32),
        voxel_size_m=0.01,
        maximum_voxels=100,
    )
    return FrozenThermalLayer(
        geometry,
        VoxelHashIndex(geometry, cell_size_m=0.08),
    )


def test_ascii_ply_is_voxelised_and_fingerprint_is_order_independent(
    tmp_path: Path,
) -> None:
    ply = tmp_path / "map.ply"
    ply.write_text(
        "\n".join(
            (
                "ply",
                "format ascii 1.0",
                "element vertex 3",
                "property float x",
                "property float y",
                "property float z",
                "property uchar red",
                "property uchar green",
                "property uchar blue",
                "element face 0",
                "property list uchar int vertex_indices",
                "end_header",
                "0.000 0 0 255 0 0",
                "0.010 0 0 255 0 0",
                "0.100 0 0 0 255 0",
            )
        )
        + "\n",
        encoding="ascii",
    )
    geometry = FixedGeometry.from_ply(
        ply,
        voxel_size_m=0.03,
        maximum_voxels=10,
    )
    reordered = FixedGeometry.from_points(
        np.asarray(
            [[0.100, 0, 0], [0.010, 0, 0], [0.000, 0, 0]],
            dtype=np.float32,
        ),
        voxel_size_m=0.03,
        maximum_voxels=10,
    )

    assert geometry.source_vertex_count == 3
    assert geometry.points.shape == (2, 3)
    assert geometry.points[0] == pytest.approx([0.005, 0.0, 0.0])
    assert geometry.fingerprint == reordered.fingerprint


def test_binary_little_endian_pcl_ply_loads_xyz(tmp_path: Path) -> None:
    ply = tmp_path / "cloud.ply"
    header = "\n".join(
        (
            "ply",
            "format binary_little_endian 1.0",
            "comment PCL generated",
            "element vertex 2",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "element face 0",
            "property list uchar int vertex_indices",
            "end_header",
            "",
        )
    ).encode("ascii")
    payload = struct.pack("<fffBBB", 1.0, 2.0, 3.0, 1, 2, 3) + struct.pack(
        "<fffBBB", -1.0, -2.0, -3.0, 4, 5, 6
    )
    ply.write_bytes(header + payload)

    np.testing.assert_allclose(
        load_ply_xyz(ply),
        np.asarray([[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]]),
    )


def test_ply_vertex_count_is_bounded_before_allocation(tmp_path: Path) -> None:
    ply = tmp_path / "huge.ply"
    ply.write_text(
        "ply\nformat ascii 1.0\nelement vertex 100\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n",
        encoding="ascii",
    )
    with pytest.raises(PlyFormatError, match="configured bound"):
        load_ply_xyz(ply, maximum_source_vertices=10)


def test_voxel_hash_matches_only_existing_geometry() -> None:
    layer = _layer([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    matched, distances = layer.index.nearest(
        np.asarray([[0.03, 0, 0], [0.50, 0, 0]], dtype=np.float32),
        0.08,
    )
    assert matched.tolist() == [0, -1]
    assert distances[0] == pytest.approx(0.03)
    assert math.isinf(float(distances[1]))


def test_voxel_hash_does_not_cache_empty_noise_and_caps_candidate_cache() -> None:
    layer = _layer(
        [(float(index), 0.0, 0.0) for index in range(20)]
    )
    layer.index.nearest(
        np.asarray(
            [[1000.0 + index, 0.0, 0.0] for index in range(100)],
            dtype=np.float32,
        ),
        0.08,
    )
    assert len(layer.index._candidate_cache) == 0

    layer.index._maximum_candidate_cache_cells = 2
    layer.index.nearest(layer.geometry.points, 0.08)
    assert len(layer.index._candidate_cache) <= 2


def test_integration_updates_matched_voxel_and_leaves_unseen_geometry_unchanged() -> None:
    layer = _layer([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    first = layer.integrate(
        np.asarray([[0.01, 0, 0], [0.50, 0, 0]], dtype=np.float32),
        np.asarray([20.0, 99.0], dtype=np.float32),
        np.asarray([0.8, 1.0], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=0.40,
        minimum_observations=1,
    )
    second = layer.integrate(
        np.asarray([[0.0, 0, 0]], dtype=np.float32),
        np.asarray([30.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=20,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )

    assert first.accepted and second.accepted
    assert first.updated_voxel_count == 1
    assert layer.geometry.points.shape == (2, 3)
    assert layer.observation_count.tolist() == [2, 0]
    assert layer.mean_c[0] == pytest.approx(25.0)
    assert layer.minimum_c[0] == pytest.approx(20.0)
    assert layer.maximum_c[0] == pytest.approx(30.0)
    assert 20.0 < layer.temperature_c[0] < 30.0
    assert math.isnan(float(layer.temperature_c[1]))
    assert layer.last_seen_ns.tolist() == [20, 0]
    assert layer.rejected_observation_count == 1


def test_static_dirty_indices_are_returned_and_coalesced_until_drained() -> None:
    layer = _layer([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    first = layer.integrate(
        np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32),
        np.asarray([20.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )
    second = layer.integrate(
        np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32),
        np.asarray([25.0, 30.0], dtype=np.float32),
        np.asarray([1.0, 0.8], dtype=np.float32),
        observed_at_ns=20,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )

    assert first.updated_voxel_indices == (0,)
    assert second.updated_voxel_indices == (0, 1)
    assert layer.pending_dirty_indices == (0, 1)
    assert layer.drain_dirty_indices() == (0, 1)
    assert layer.pending_dirty_indices == ()
    assert layer.drain_dirty_indices() == ()


def test_rejected_static_frame_does_not_add_dirty_indices() -> None:
    layer = _layer([(0.0, 0.0, 0.0)])
    result = layer.integrate(
        np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32),
        np.asarray([20.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )

    assert not result.accepted
    assert result.updated_voxel_indices == ()
    assert layer.pending_dirty_indices == ()


def test_identical_static_display_values_are_not_marked_dirty_again() -> None:
    layer = _layer([(0.0, 0.0, 0.0)])
    kwargs = dict(
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )
    layer.integrate(
        np.zeros((1, 3), dtype=np.float32),
        np.asarray([20.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        **kwargs,
    )
    assert layer.drain_dirty_indices() == (0,)

    repeated = layer.integrate(
        np.zeros((1, 3), dtype=np.float32),
        np.asarray([20.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        **{**kwargs, "observed_at_ns": 20},
    )

    assert repeated.updated_voxel_count == 0
    assert repeated.updated_voxel_indices == ()
    assert layer.pending_dirty_indices == ()


def test_same_voxel_frame_mean_does_not_hide_raw_temperature_extrema() -> None:
    layer = _layer([(0.0, 0.0, 0.0)])
    first = layer.integrate(
        np.zeros((2, 3), dtype=np.float32),
        np.asarray([20.0, 80.0], dtype=np.float32),
        np.ones(2, dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )

    assert first.accepted
    assert layer.observation_count[0] == 1
    assert layer.temperature_c[0] == pytest.approx(50.0)
    assert layer.mean_c[0] == pytest.approx(50.0)
    assert layer.minimum_c[0] == pytest.approx(20.0)
    assert layer.maximum_c[0] == pytest.approx(80.0)

    second = layer.integrate(
        np.zeros((2, 3), dtype=np.float32),
        np.asarray([10.0, 100.0], dtype=np.float32),
        np.ones(2, dtype=np.float32),
        observed_at_ns=20,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )

    assert second.accepted
    assert layer.observation_count[0] == 2
    assert layer.mean_c[0] == pytest.approx(52.5)
    assert layer.minimum_c[0] == pytest.approx(10.0)
    assert layer.maximum_c[0] == pytest.approx(100.0)


def test_low_match_ratio_rejects_whole_frame_without_partial_update() -> None:
    layer = _layer([(0.0, 0.0, 0.0)])
    result = layer.integrate(
        np.asarray([[0.01, 0, 0], [1.0, 0, 0]], dtype=np.float32),
        np.asarray([50.0, 80.0], dtype=np.float32),
        np.asarray([1.0, 1.0], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=0.75,
        minimum_observations=1,
    )
    assert not result.accepted
    assert result.reason == "match_ratio_below_threshold"
    assert layer.observed_voxel_count == 0
    assert layer.rejected_observation_count == 2


def test_range_consistency_rejects_closer_dynamic_point_and_keeps_wall_unseen() -> None:
    layer = _layer([(1.0, 0.0, 0.0)])
    result = layer.integrate(
        np.asarray([[0.95, 0.0, 0.0]], dtype=np.float32),
        np.asarray([36.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=10,
        association_radius_m=0.08,
        minimum_match_ratio=1.0,
        minimum_observations=1,
        sensor_origin=np.asarray([0.0, 0.0, 0.0]),
        maximum_surface_range_residual_m=0.04,
    )
    assert not result.accepted
    assert result.surface_range_rejected_count == 1
    assert layer.surface_range_rejected_count == 1
    assert layer.observed_voxel_count == 0
    assert math.isnan(float(layer.temperature_c[0]))


def test_range_consistency_accepts_observation_on_fixed_surface() -> None:
    layer = _layer([(1.0, 0.0, 0.0)])
    result = layer.integrate(
        np.asarray([[0.97, 0.0, 0.0]], dtype=np.float32),
        np.asarray([36.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=10,
        association_radius_m=0.08,
        minimum_match_ratio=1.0,
        minimum_observations=1,
        sensor_origin=np.asarray([0.0, 0.0, 0.0]),
        maximum_surface_range_residual_m=0.04,
    )
    assert result.accepted
    assert result.surface_range_rejected_count == 0
    assert layer.observed_voxel_count == 1


def test_refreshing_only_visible_voxel_keeps_unseen_voxel_state_exactly() -> None:
    layer = _layer([(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    layer.integrate(
        np.asarray([[1.0, 0, 0], [2.0, 0, 0]], dtype=np.float32),
        np.asarray([20.0, 30.0], dtype=np.float32),
        np.asarray([1.0, 1.0], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
        sensor_origin=np.zeros(3),
    )
    unseen_before = (
        float(layer.temperature_c[1]),
        int(layer.observation_count[1]),
        int(layer.last_seen_ns[1]),
    )
    layer.integrate(
        np.asarray([[1.0, 0, 0]], dtype=np.float32),
        np.asarray([40.0], dtype=np.float32),
        np.asarray([1.0], dtype=np.float32),
        observed_at_ns=20,
        minimum_match_ratio=1.0,
        minimum_observations=1,
        sensor_origin=np.zeros(3),
    )
    assert layer.temperature_c[0] > 20.0
    assert layer.observation_count[0] == 2
    assert layer.last_seen_ns[0] == 20
    assert (
        float(layer.temperature_c[1]),
        int(layer.observation_count[1]),
        int(layer.last_seen_ns[1]),
    ) == unseen_before


def test_atomic_state_round_trip_and_geometry_fingerprint_gate(
    tmp_path: Path,
) -> None:
    layer = _layer([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    layer.integrate(
        np.asarray([[0.0, 0, 0]], dtype=np.float32),
        np.asarray([42.5], dtype=np.float32),
        np.asarray([0.9], dtype=np.float32),
        observed_at_ns=123,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )
    state = tmp_path / "thermal_layer.npz"
    layer.save_atomic(state)

    restored = _layer([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    restored.restore(state)
    assert restored.restored
    assert restored.observed_voxel_count == 1
    assert restored.temperature_c[0] == pytest.approx(42.5)
    assert restored.last_seen_ns[0] == 123
    assert restored.latest_observation_ns == 123
    assert not list(tmp_path.glob(".*.tmp"))

    different = _layer([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    with pytest.raises(ThermalStateError, match="fingerprint"):
        different.restore(state)
    assert different.observed_voxel_count == 0


def test_optional_alignment_is_bounded_and_does_not_modify_geometry() -> None:
    points = [(index * 0.10, 0.0, 0.0) for index in range(25)]
    layer = _layer(points)
    observations = layer.geometry.points + np.asarray([0.04, 0.0, 0.0])
    original = layer.geometry.points.copy()

    accepted = estimate_bounded_translation(
        layer.index,
        observations,
        search_radius_m=0.08,
        maximum_translation_m=0.05,
        minimum_matches=20,
    )
    rejected = estimate_bounded_translation(
        layer.index,
        observations,
        search_radius_m=0.08,
        maximum_translation_m=0.02,
        minimum_matches=20,
    )
    assert accepted.accepted
    assert accepted.translation == pytest.approx([-0.04, 0.0, 0.0], abs=1e-5)
    assert not rejected.accepted
    assert np.array_equal(layer.geometry.points, original)


def test_keyframe_policy_uses_ten_centimetres_or_six_degrees() -> None:
    origin = (
        np.zeros(3, dtype=np.float64),
        np.asarray([0.0, 0.0, 0.0, 1.0]),
    )
    assert not is_keyframe_pose(
        origin,
        np.asarray([0.099, 0.0, 0.0]),
        origin[1],
    )
    assert is_keyframe_pose(
        origin,
        np.asarray([0.10, 0.0, 0.0]),
        origin[1],
    )
    angle = math.radians(6.0)
    rotation = np.asarray([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
    assert is_keyframe_pose(origin, origin[0], rotation)


def test_localization_gate_requires_three_stable_samples_and_resets_on_jump() -> None:
    gate = LocalizationStabilityGate(
        required_samples=3,
        maximum_translation_m=0.05,
        maximum_rotation_rad=math.radians(3.0),
    )
    identity = np.asarray([0.0, 0.0, 0.0, 1.0])
    assert not gate.observe(np.asarray([0.00, 0.0, 0.0]), identity)
    assert not gate.observe(np.asarray([0.02, 0.0, 0.0]), identity)
    assert not gate.observe(np.asarray([0.20, 0.0, 0.0]), identity)
    assert gate.sample_count == 1
    assert not gate.observe(np.asarray([0.21, 0.0, 0.0]), identity)
    assert gate.observe(np.asarray([0.19, 0.0, 0.0]), identity)


def test_stationary_refresh_and_rejected_retry_cadence() -> None:
    assert not thermal_update_due(
        motion_keyframe=False,
        seconds_since_success=4.9,
        seconds_since_rejection=4.0,
        stationary_refresh_interval_sec=5.0,
        rejected_frame_retry_sec=2.0,
    )
    assert thermal_update_due(
        motion_keyframe=False,
        seconds_since_success=5.0,
        seconds_since_rejection=4.0,
        stationary_refresh_interval_sec=5.0,
        rejected_frame_retry_sec=2.0,
    )
    assert not thermal_update_due(
        motion_keyframe=True,
        seconds_since_success=99.0,
        seconds_since_rejection=1.0,
        stationary_refresh_interval_sec=5.0,
        rejected_frame_retry_sec=2.0,
    )


def test_fixed_publish_subset_never_churns_as_observations_grow() -> None:
    subset = fixed_stride_indices(250_000, 100_000)
    first_observed = np.arange(0, 50_000, dtype=np.int32)
    later_observed = np.arange(0, 200_000, dtype=np.int32)
    first_visible = np.intersect1d(first_observed, subset)
    later_visible = np.intersect1d(later_observed, subset)
    assert subset.shape[0] <= 100_000
    assert np.all(np.isin(first_visible, later_visible))
    assert np.array_equal(subset, fixed_stride_indices(250_000, 100_000))
