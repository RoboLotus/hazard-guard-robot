from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import struct

from sensor_msgs.msg import PointCloud2


SCRIPT = Path(__file__).parents[1] / "scripts" / "adaptive_cloud_guard.py"
REAL_LAUNCH = Path(__file__).parents[1] / "launch" / "rtabmap_real.launch.py"
SPEC = spec_from_file_location("adaptive_cloud_guard", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def load_sample(value):
    return {
        "cpu": float(value),
        "gpu": float(value),
        "memory": float(value),
        "temperature": float(value),
        "disk": float(value),
    }


def test_policy_escalates_after_window_and_recovers_with_dwell():
    policy = MODULE.LoadPolicy(window_samples=3, recovery_seconds=10.0)
    assert policy.add(load_sample(100), 0.0)[0] == "normal"
    assert policy.add(load_sample(100), 1.0)[0] == "normal"
    assert policy.add(load_sample(100), 2.0)[0] == "critical"

    policy.add(load_sample(0), 3.0)
    policy.add(load_sample(0), 4.0)
    assert policy.add(load_sample(0), 5.0)[0] == "critical"
    assert policy.add(load_sample(0), 16.0)[0] == "normal"


def make_cloud(width, height, row_step, values):
    cloud = PointCloud2()
    cloud.width = width
    cloud.height = height
    cloud.point_step = 4
    cloud.row_step = row_step
    cloud.data = b"".join(struct.pack("<I", value) for value in values)
    return cloud


def unpack_values(cloud):
    return [
        struct.unpack_from("<I", cloud.data, offset)[0]
        for offset in range(0, len(cloud.data), cloud.point_step)
    ]


def test_even_sampling_caps_unorganized_cloud():
    cloud = make_cloud(width=6, height=1, row_step=24, values=range(6))
    sampled = MODULE.evenly_sample_cloud(cloud, 3)

    assert sampled.width == 3
    assert sampled.height == 1
    assert sampled.row_step == 12
    assert unpack_values(sampled) == [0, 2, 4]


def test_even_sampling_handles_padded_organized_rows():
    # Each two-point row has one four-byte padding slot.
    cloud = make_cloud(
        width=2,
        height=2,
        row_step=12,
        values=[0, 1, 99, 2, 3, 99],
    )
    sampled = MODULE.evenly_sample_cloud(cloud, 2)

    assert unpack_values(sampled) == [0, 2]


def test_small_cloud_is_forwarded_without_copy():
    cloud = make_cloud(width=2, height=1, row_step=8, values=[10, 20])
    assert MODULE.evenly_sample_cloud(cloud, 3) is cloud


def test_real_map_assembler_publishes_circular_buffer_immediately():
    launch_source = REAL_LAUNCH.read_text(encoding="utf-8")
    assert '"circular_buffer": True' in launch_source
