import math

import pytest

from hazard_guard_thermal_analysis.projection import CameraIntrinsics, RigidTransform, fuse_depth_and_thermal


def test_identity_projection_attaches_temperature_and_source_pixel() -> None:
    camera = CameraIntrinsics(3, 3, 1.0, 1.0, 1.0, 1.0)
    points = fuse_depth_and_thermal(
        [1.0] * 9,
        camera,
        [20.0 + index for index in range(9)],
        camera,
        RigidTransform(),
        stride=1,
    )
    assert len(points) == 9
    center = points[4]
    assert (center.x, center.y, center.z) == pytest.approx((0.0, 0.0, 1.0))
    assert center.temperature_c == pytest.approx(24.0)
    assert (center.pixel_u, center.pixel_v) == pytest.approx((1.0, 1.0))
    assert 0.0 < center.confidence <= 1.0


def test_rigid_transform_rotates_and_translates() -> None:
    half = math.sqrt(0.5)
    transform = RigidTransform(tx=1.0, qz=half, qw=half)
    assert transform.apply(1.0, 0.0, 0.0) == pytest.approx((1.0, 1.0, 0.0))


def test_intrinsics_can_fall_back_to_horizontal_fov() -> None:
    intrinsics = CameraIntrinsics.from_horizontal_fov(160, 120, 57.0)
    assert intrinsics.fx == pytest.approx(intrinsics.fy)
    assert intrinsics.fx > 0.0
    assert intrinsics.cx == pytest.approx(79.5)
    assert intrinsics.cy == pytest.approx(59.5)


def test_plumb_bob_distortion_is_applied_to_thermal_projection() -> None:
    undistorted = CameraIntrinsics(160, 120, 100.0, 100.0, 80.0, 60.0)
    distorted = CameraIntrinsics(
        160,
        120,
        100.0,
        100.0,
        80.0,
        60.0,
        distortion=(0.2, 0.0, 0.0, 0.0, 0.0),
    )
    plain_u, plain_v = undistorted.project(0.4, 0.2, 1.0)
    curved_u, curved_v = distorted.project(0.4, 0.2, 1.0)
    assert curved_u > plain_u
    assert curved_v > plain_v


def test_projection_rejects_invalid_depth_and_outside_fov() -> None:
    camera = CameraIntrinsics(2, 1, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [float("nan"), 1.0], camera, [25.0, 30.0], camera,
        RigidTransform(tx=10.0), stride=1,
    )
    assert points == []


def test_bilinear_sampling_interpolates_at_projected_subpixel() -> None:
    depth_camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    thermal_camera = CameraIntrinsics(2, 2, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [1.0],
        depth_camera,
        [10.0, 20.0, 30.0, 40.0],
        thermal_camera,
        RigidTransform(tx=0.25, ty=0.5),
        stride=1,
        thermal_sampling_mode="bilinear",
    )
    assert len(points) == 1
    assert points[0].temperature_c == pytest.approx(22.5)
    assert (points[0].pixel_u, points[0].pixel_v) == pytest.approx((0.25, 0.5))


def test_bilinear_sampling_renormalizes_valid_neighbours() -> None:
    depth_camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    thermal_camera = CameraIntrinsics(2, 2, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [1.0],
        depth_camera,
        [10.0, float("nan"), 30.0, float("inf")],
        thermal_camera,
        RigidTransform(tx=0.5, ty=0.5),
        stride=1,
    )
    assert len(points) == 1
    assert points[0].temperature_c == pytest.approx(20.0)


def test_bilinear_sampling_handles_image_boundary() -> None:
    depth_camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    thermal_camera = CameraIntrinsics(2, 2, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [1.0],
        depth_camera,
        [10.0, 20.0, 30.0, 40.0],
        thermal_camera,
        RigidTransform(tx=1.0, ty=1.0),
        stride=1,
    )
    assert len(points) == 1
    assert points[0].temperature_c == pytest.approx(40.0)


def test_bilinear_sampling_skips_when_no_valid_neighbour_exists() -> None:
    camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    assert fuse_depth_and_thermal(
        [1.0], camera, [float("nan")], camera, RigidTransform(), stride=1
    ) == []


def test_nearest_sampling_mode_preserves_rounded_pixel_sampling() -> None:
    depth_camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    thermal_camera = CameraIntrinsics(2, 2, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [1.0],
        depth_camera,
        [10.0, 20.0, 30.0, 40.0],
        thermal_camera,
        RigidTransform(tx=0.6, ty=0.6),
        stride=1,
        thermal_sampling_mode="nearest",
    )
    assert len(points) == 1
    assert points[0].temperature_c == pytest.approx(40.0)
    assert (points[0].pixel_u, points[0].pixel_v) == pytest.approx((1.0, 1.0))


def test_invalid_thermal_sampling_mode_is_rejected() -> None:
    camera = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="thermal_sampling_mode"):
        fuse_depth_and_thermal(
            [1.0], camera, [20.0], camera, RigidTransform(),
            thermal_sampling_mode="cubic",
        )
