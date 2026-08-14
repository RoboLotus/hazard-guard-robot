"""The projection chain thermal_cloud.py builds its map from.

Everything here is the part that can be wrong without anyone noticing: a
half-pixel offset in back-projection, a voxel key that rounds the wrong way, a
colour ramp that runs backwards. The ROS plumbing around it fails loudly.
"""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import struct

import numpy as np
from builtin_interfaces.msg import Time


SCRIPT = Path(__file__).parents[1] / "scripts" / "thermal_cloud.py"
SPEC = spec_from_file_location("thermal_cloud", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

# fx, fy, cx, cy for a 640 x 480 camera, laid out the way CameraInfo.k is.
K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]


def test_back_projected_points_reproject_to_their_own_pixels():
    """A point put through the same camera it came from lands where it started."""
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    points = MODULE.back_project(depth, K, stride=160, near=0.25, far=5.0)
    assert points.shape[0] == (480 // 160) * (640 // 160)

    u, v, in_front = MODULE.project(points, K)
    assert in_front.all()
    # The sampled pixels are exactly the stride grid.
    assert sorted(set(np.round(u).astype(int))) == [0, 160, 320, 480]
    assert sorted(set(np.round(v).astype(int))) == [0, 160, 320]


def test_back_project_drops_holes_and_out_of_range_depth():
    depth = np.array([[0.0, np.inf], [0.1, 9.0]], dtype=np.float32)
    assert MODULE.back_project(depth, K, 1, 0.25, 5.0).shape[0] == 0


def test_project_marks_points_behind_the_camera():
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]], dtype=np.float32)
    _, _, in_front = MODULE.project(points, K)
    assert in_front.tolist() == [True, False]


def test_voxel_average_groups_by_cell_and_averages_inside_it():
    points = np.array([
        [0.01, 0.01, 0.01],   # same 5 cm voxel
        [0.04, 0.04, 0.04],   # same 5 cm voxel
        [0.31, 0.0, 0.0],     # a different one
        [-0.01, 0.0, 0.0],    # negative coordinates floor away from zero
    ])
    keys, temperatures = MODULE.voxel_average(
        points, np.array([20.0, 30.0, 50.0, 11.0]), 0.05
    )
    cells = {tuple(key): value for key, value in zip(keys.tolist(), temperatures.tolist())}
    assert cells[(0, 0, 0)] == 25.0
    assert cells[(6, 0, 0)] == 50.0
    assert cells[(-1, 0, 0)] == 11.0


def test_colour_ramp_runs_cold_blue_to_hot_red_and_clamps():
    colors = MODULE.temperature_colors(
        np.array([-40.0, 10.0, 60.0, 300.0]), low=10.0, high=60.0
    )
    red = (colors >> 16) & 0xFF
    blue = colors & 0xFF
    assert blue[1] > red[1]        # at the cold end
    assert red[2] > blue[2]        # at the hot end
    assert colors[0] == colors[1]  # below the window clamps to the cold end
    assert colors[3] == colors[2]  # above it clamps to the hot end


def test_cloud_message_is_readable_the_way_the_console_reads_it():
    """The console unpacks the rgb field as a uint32 at offset 12."""
    points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    message = MODULE.cloud_message(points, np.array([0x123456]), "map", Time())

    assert message.point_step == 16
    assert message.width == 1 and message.height == 1
    assert [field.name for field in message.fields] == ["x", "y", "z", "rgb"]
    x, y, z = struct.unpack_from("<fff", message.data, 0)
    assert (x, y, z) == (1.0, 2.0, 3.0)
    assert struct.unpack_from("<I", message.data, 12)[0] == 0x123456
