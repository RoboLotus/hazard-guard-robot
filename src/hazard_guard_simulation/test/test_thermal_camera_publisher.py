from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "thermal_camera_publisher.py"
SPEC = importlib.util.spec_from_file_location("thermal_camera_publisher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_sdk_width_major_pixels_are_transposed_to_ros_rows() -> None:
    sdk = np.array([[1, 2], [3, 4], [5, 6]], dtype=np.float64)

    image = MODULE.sdk_pixels_to_image(sdk, width=3, height=2)

    assert image.dtype == np.uint16
    assert image.tolist() == [[1, 3, 5], [2, 4, 6]]


def test_estimated_camera_info_uses_requested_frame_and_resolution() -> None:
    info = MODULE.estimated_camera_info(
        width=160,
        height=120,
        frame_id="thermal_camera_optical_frame",
        horizontal_fov_deg=57.0,
    )

    assert info.header.frame_id == "thermal_camera_optical_frame"
    assert (info.width, info.height) == (160, 120)
    assert info.k[0] > 0.0
    assert info.k[4] == info.k[0]
    assert info.k[2] == 80.0
    assert info.k[5] == 60.0


def test_relative_color_bounds_ignore_single_pixel_outliers() -> None:
    temperatures = np.full((10, 10), 25.0, dtype=np.float32)
    temperatures[:, 5:] = 35.0
    temperatures[0, 0] = -100.0
    temperatures[-1, -1] = 500.0

    low, high = MODULE.color_scale_bounds(
        temperatures,
        mode="relative",
        fixed_low_c=10.0,
        fixed_high_c=60.0,
        relative_low_percentile=2.0,
        relative_high_percentile=98.0,
    )

    assert low == 25.0
    assert high == 35.0


def test_fixed_color_bounds_preserve_absolute_temperature_scale() -> None:
    temperatures = np.array([[20.0, 30.0]], dtype=np.float32)

    low, high = MODULE.color_scale_bounds(
        temperatures,
        mode="fixed",
        fixed_low_c=10.0,
        fixed_high_c=60.0,
        relative_low_percentile=2.0,
        relative_high_percentile=98.0,
    )

    assert (low, high) == (10.0, 60.0)


def test_sdk_rgb_bitmap_is_converted_to_bgr() -> None:
    bitmap = bytes([10, 20, 30, 40, 50, 60])

    image = MODULE.sdk_bitmap_to_bgr(bitmap, width=2, height=1)

    assert image.shape == (1, 2, 3)
    assert image.tolist() == [[[30, 20, 10], [60, 50, 40]]]


def test_physical_extrinsic_yaml_builds_rgb_parent_thermal_child(tmp_path) -> None:
    path = tmp_path / "extrinsic.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "parent_frame_id": "ascamera_hp60c_color_0",
                "child_frame_id": "thermal_camera_optical_frame",
                "translation": {"x": 0.0, "y": 0.068, "z": 0.0},
                "rotation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ),
        encoding="utf-8",
    )

    message = MODULE.calibrated_extrinsic(
        path, "thermal_camera_optical_frame"
    )

    assert message.header.frame_id == "ascamera_hp60c_color_0"
    assert message.child_frame_id == "thermal_camera_optical_frame"
    assert message.transform.translation.y == 0.068
