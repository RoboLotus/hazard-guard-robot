from __future__ import annotations

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "physical_thermal_calibration.py"
)
SPEC = spec_from_file_location("physical_thermal_calibration", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_six_by_four_square_board_has_five_by_three_inner_corners() -> None:
    points = MODULE.board_object_points()

    assert points.shape == (15, 3)
    assert points.dtype == np.float32
    assert points[0].tolist() == [0.0, 0.0, 0.0]
    assert np.allclose(points[-1], [0.19, 0.095, 0.0])


def test_thermal_normalization_is_robust_to_outliers() -> None:
    image = np.full((20, 20), 29500, dtype=np.uint16)
    image[:, 10:] = 30500
    image[0, 0] = 0
    image[-1, -1] = 65535

    normalized = MODULE.normalize_thermal(image)

    assert normalized.dtype == np.uint8
    assert normalized[5, 2] == 0
    assert normalized[5, 17] == 255


def test_depth_preview_converts_uint16_millimetres_and_ignores_zero() -> None:
    depth = np.array([[0, 1000], [2000, 3000]], dtype=np.uint16)

    preview, label = MODULE.depth_preview(depth, "16UC1")

    assert preview.shape == (2, 2, 3)
    assert preview[0, 0].tolist() == [0, 0, 0]
    assert label.endswith(" m")


def test_preview_panel_preserves_requested_dimensions() -> None:
    image = np.zeros((120, 160, 3), dtype=np.uint8)

    panel = MODULE.preview_panel(image, "THERMAL", "corners OK", 300, 240)

    assert panel.shape == (240, 300, 3)


def test_corner_order_is_reversed_when_cameras_disagree() -> None:
    rgb = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    thermal = np.array([[12.0, 0.0], [11.0, 0.0], [10.0, 0.0]])

    aligned = MODULE.align_corner_order(rgb, thermal)

    assert aligned.tolist() == [[10.0, 0.0], [11.0, 0.0], [12.0, 0.0]]


def test_transform_inverse_round_trip() -> None:
    rotation, _ = cv2.Rodrigues(np.array([0.1, -0.2, 0.05]))
    translation = np.array([0.07, -0.01, 0.02])
    inverse_rotation, inverse_translation = MODULE.invert_transform(
        rotation, translation
    )
    point = np.array([0.2, -0.3, 1.4])

    in_target = rotation @ point + translation
    recovered = inverse_rotation @ in_target + inverse_translation

    assert np.allclose(recovered, point)
    quaternion = MODULE.quaternion_from_rotation(inverse_rotation)
    assert np.isclose(np.linalg.norm(quaternion), 1.0)


def test_thermal_camera_yaml_is_standard_ros_calibration() -> None:
    matrix = np.array(
        [[147.0, 0.0, 80.0], [0.0, 148.0, 60.0], [0.0, 0.0, 1.0]]
    )

    document = yaml.safe_load(
        MODULE.thermal_camera_yaml(160, 120, matrix, np.zeros((5, 1)))
    )

    assert document["image_width"] == 160
    assert document["image_height"] == 120
    assert document["camera_matrix"]["data"] == matrix.reshape(-1).tolist()
    assert len(document["projection_matrix"]["data"]) == 12
