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


def test_projection_rejects_invalid_depth_and_outside_fov() -> None:
    camera = CameraIntrinsics(2, 1, 1.0, 1.0, 0.0, 0.0)
    points = fuse_depth_and_thermal(
        [float("nan"), 1.0], camera, [25.0, 30.0], camera,
        RigidTransform(tx=10.0), stride=1,
    )
    assert points == []
