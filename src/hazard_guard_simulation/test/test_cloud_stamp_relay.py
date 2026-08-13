from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time


SCRIPT = Path(__file__).parents[1] / "scripts" / "cloud_stamp_relay.py"
SPEC = spec_from_file_location("cloud_stamp_relay", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_preserve_mode_copies_original_stamp():
    original = Time(sec=12, nanosec=345)

    result = MODULE.adjusted_stamp(original, "preserve", 99.0)

    assert (result.sec, result.nanosec) == (12, 345)
    assert result is not original


def test_latest_mode_uses_zero_stamp_for_latest_tf():
    result = MODULE.adjusted_stamp(Time(sec=12, nanosec=345), "latest", 0.0)

    assert (result.sec, result.nanosec) == (0, 0)


@pytest.mark.parametrize(
    ("offset", "expected"),
    [
        (0.25, (11, 150_000_000)),
        (-0.25, (10, 650_000_000)),
        (-20.0, (0, 0)),
    ],
)
def test_offset_mode_normalizes_and_clamps_stamp(offset, expected):
    original = Time(sec=10, nanosec=900_000_000)

    result = MODULE.adjusted_stamp(original, "offset", offset)

    assert (result.sec, result.nanosec) == expected


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match="unsupported"):
        MODULE.adjusted_stamp(Time(), "guess", 0.0)


@pytest.mark.parametrize("offset", [float("nan"), float("inf")])
def test_non_finite_offset_is_rejected(offset):
    with pytest.raises(ValueError, match="finite"):
        MODULE.adjusted_stamp(Time(), "offset", offset)


def test_offset_beyond_ros_time_range_is_rejected():
    with pytest.raises(ValueError, match="range"):
        MODULE.adjusted_stamp(Time(), "offset", 3_000_000_000.0)
