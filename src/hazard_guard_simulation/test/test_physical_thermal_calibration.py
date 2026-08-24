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


def test_corner_rms_detects_duplicate_and_moved_board_poses() -> None:
    original = np.zeros((15, 2), dtype=np.float32)
    duplicate = original + 0.25
    moved = original + 3.0

    assert MODULE.corner_rms(original, duplicate) < 0.5
    assert MODULE.corner_rms(original, moved) == 3.0


def test_thermal_capture_guide_uses_nine_centroid_cells() -> None:
    corners = np.array([[120.0, 90.0], [130.0, 100.0]], dtype=np.float32)
    image = np.zeros((120, 160, 3), dtype=np.uint8)

    cell = MODULE.image_grid_cell(corners, (160, 120))
    MODULE.draw_thirds_grid(image)

    assert cell == (2, 2)
    assert np.any(image)


def test_online_pose_gate_rejects_gross_translation_outlier() -> None:
    previous = []
    for index in range(8):
        previous.append(
            MODULE.CapturedView(
                index=index + 1,
                path=Path(f"view_{index + 1:03d}.npz"),
                rgb_corners=np.zeros((15, 2), dtype=np.float32),
                thermal_corners=np.zeros((15, 2), dtype=np.float32),
                object_points=np.zeros((15, 3), dtype=np.float32),
                rgb_size=(640, 480),
                thermal_size=(160, 120),
                rgb_frame="rgb",
                thermal_frame="thermal",
                relative_rotation=np.eye(3),
                relative_translation=np.array([0.06, 0.0, index * 0.0001]),
            )
        )

    reason = MODULE.pose_outlier_reason(
        np.eye(3), np.array([0.25, 0.0, 0.0]), previous
    )

    assert reason is not None
    assert reason.startswith("T 이상치")


def test_manual_capture_is_the_default() -> None:
    arguments = MODULE.parser().parse_args([])

    assert arguments.auto_capture_interval_sec == 0.0
    assert arguments.stable_duration_sec == 0.8
    assert arguments.maximum_solver_views == 40


def test_diverse_selection_caps_views_and_covers_nine_image_cells() -> None:
    rgb_info = MODULE.CameraInfo()
    rgb_info.width = 640
    rgb_info.height = 480
    rgb_info.k = [580.0, 0.0, 320.0, 0.0, 580.0, 240.0, 0.0, 0.0, 1.0]
    rgb_info.d = [0.0] * 5
    object_points = MODULE.board_object_points()
    views = []
    index = 1
    for cell_y in range(3):
        for cell_x in range(3):
            for variant in range(6):
                rotation_vector = np.array([
                    0.03 * (variant - 2),
                    0.04 * (variant - 2),
                    0.01 * variant,
                ])
                translation = np.array([
                    0.01 * (cell_x - 1),
                    0.01 * (cell_y - 1),
                    0.4 + 0.04 * variant,
                ])
                rgb_corners = cv2.projectPoints(
                    object_points,
                    rotation_vector,
                    translation,
                    np.asarray(rgb_info.k).reshape(3, 3),
                    np.asarray(rgb_info.d),
                )[0].reshape(-1, 2).astype(np.float32)
                centre = np.array([
                    (cell_x + 0.5) * 160.0 / 3.0,
                    (cell_y + 0.5) * 120.0 / 3.0,
                ])
                thermal_corners = (
                    np.mgrid[0:5, 0:3].T.reshape(-1, 2) * [5.0, 5.0]
                    + centre - [10.0, 5.0]
                ).astype(np.float32)
                views.append(
                    MODULE.CapturedView(
                        index=index,
                        path=Path(f"view_{index:03d}.npz"),
                        rgb_corners=rgb_corners,
                        thermal_corners=thermal_corners,
                        object_points=object_points,
                        rgb_size=(640, 480),
                        thermal_size=(160, 120),
                        rgb_frame="rgb",
                        thermal_frame="thermal",
                        relative_rotation=np.eye(3),
                        relative_translation=np.array([0.06, 0.0, 0.0]),
                    )
                )
                index += 1

    selected, rejected, coverage = MODULE.select_diverse_views(
        views, rgb_info, maximum_views=40
    )

    assert len(selected) == 40
    assert coverage == {"occupied_cells": 9, "total_cells": 9}
    assert len(rejected) == 14
    reasons = {item["reason"] for item in rejected}
    assert "diversity_limit" in reasons
    assert any(reason.startswith("duplicate_of_view_") for reason in reasons)


def test_reprojection_limit_has_robust_floor_and_absolute_ceiling() -> None:
    assert MODULE.reprojection_outlier_limit(
        {1: 0.8, 2: 0.9, 3: 1.0}, 4.0
    ) == 2.0
    assert MODULE.reprojection_outlier_limit(
        {1: 1.0, 2: 2.0, 3: 20.0}, 4.0
    ) == 4.0


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
