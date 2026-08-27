from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import numpy as np
import yaml


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "physical_calibration_validator.py"
)
SPEC = spec_from_file_location("physical_calibration_validator", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_uint16_depth_is_millimetres() -> None:
    depth = np.array([[0, 1000, 2500]], dtype=np.uint16)

    result = MODULE.depth_in_metres(depth, "16UC1")

    assert result.tolist() == [[0.0, 1.0, 2.5]]


def test_centre_depth_ignores_invalid_pixels() -> None:
    depth = np.zeros((21, 21), dtype=np.float32)
    depth[8:13, 8:13] = 1.75

    assert MODULE.median_centre_depth(depth) == 1.75


def test_saved_measurement_projects_rgb_origin_into_thermal(tmp_path) -> None:
    path = tmp_path / "extrinsic.yaml"
    path.write_text(
        yaml.safe_dump({
            "parent_frame_id": "rgb",
            "child_frame_id": "thermal",
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "measurement": {
                "direction": "thermal_from_rgb",
                "translation_m": [0.1, 0.0, 0.0],
                "rotation_matrix": np.eye(3).reshape(-1).tolist(),
            },
        }),
        encoding="utf-8",
    )
    rotation, translation, parent, child = MODULE.load_thermal_from_rgb(path)
    matrix = np.array([[100.0, 0.0, 80.0], [0.0, 100.0, 60.0], [0.0, 0.0, 1.0]])

    pixel = MODULE.project_point(
        np.array([0.0, 0.0, 1.0]), rotation, translation,
        matrix, np.zeros((5, 1)),
    )

    assert parent == "rgb" and child == "thermal"
    assert np.allclose(pixel, [90.0, 60.0])


def test_ros_parent_child_pose_is_inverted_for_projection(tmp_path) -> None:
    path = tmp_path / "extrinsic.yaml"
    path.write_text(
        yaml.safe_dump({
            "parent_frame_id": "rgb",
            "child_frame_id": "thermal",
            "translation": {"x": -0.1, "y": 0.0, "z": 0.0},
            "rotation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }),
        encoding="utf-8",
    )

    rotation, translation, _, _ = MODULE.load_thermal_from_rgb(path)

    assert np.allclose(rotation, np.eye(3))
    assert np.allclose(translation, [0.1, 0.0, 0.0])
