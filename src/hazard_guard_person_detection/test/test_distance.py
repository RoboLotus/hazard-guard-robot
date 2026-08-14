import numpy as np
import pytest

from hazard_guard_person_detection.distance import (
    depth_image_to_metres,
    estimate_bbox_distance,
)


def test_uses_robust_median_and_rejects_invalid_values():
    depth = np.full((12, 12), 2.0, dtype=np.float32)
    depth[4, 4] = 0.0
    depth[4, 5] = np.nan
    depth[5, 4] = np.inf
    depth[5, 5] = 7.5

    estimate = estimate_bbox_distance(
        depth,
        (2, 2, 10, 10),
        central_roi_ratio=0.75,
        minimum_valid_samples=5,
    )

    assert estimate.valid
    assert estimate.distance_m == pytest.approx(2.0)
    assert estimate.valid_sample_count >= 5


def test_clips_bbox_to_image_and_rejects_out_of_range_depth():
    depth = np.full((8, 8), 20.0, dtype=np.float32)
    estimate = estimate_bbox_distance(
        depth,
        (-10, -10, 20, 20),
        maximum_distance_m=8.0,
        minimum_valid_samples=1,
    )
    assert not estimate.valid
    assert estimate.distance_m == 0.0


def test_requires_enough_valid_samples():
    depth = np.zeros((10, 10), dtype=np.float32)
    depth[5, 5] = 1.4
    estimate = estimate_bbox_distance(depth, (0, 0, 10, 10), minimum_valid_samples=2)
    assert not estimate.valid
    assert estimate.valid_sample_count == 1


def test_converts_millimetre_depth_encoding_to_metres():
    depth = np.array([[500, 1250]], dtype=np.uint16)
    converted = depth_image_to_metres(depth, "16UC1")
    np.testing.assert_allclose(converted, np.array([[0.5, 1.25]], dtype=np.float32))


def test_rejects_invalid_roi_ratio():
    with pytest.raises(ValueError):
        estimate_bbox_distance(np.ones((2, 2)), (0, 0, 2, 2), central_roi_ratio=0.0)
