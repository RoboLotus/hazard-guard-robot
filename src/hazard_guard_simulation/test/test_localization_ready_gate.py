from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
import yaml


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "localization_ready_gate.py"
)
SPEC = spec_from_file_location("localization_ready_gate", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def write_map(tmp_path: Path, image: str = "map.pgm") -> Path:
    map_path = tmp_path / "map.yaml"
    map_path.write_text(
        yaml.safe_dump(
            {
                "image": image,
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
            }
        ),
        encoding="utf-8",
    )
    return map_path


def test_validate_map_files_resolves_relative_image(tmp_path):
    image_path = tmp_path / "map.pgm"
    image_path.write_bytes(b"P5\n1 1\n255\n\x00")
    map_path = write_map(tmp_path)

    resolved_map, resolved_image = MODULE.validate_map_files(str(map_path))

    assert resolved_map == map_path.resolve()
    assert resolved_image == image_path.resolve()


def test_validate_map_files_rejects_missing_yaml(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        MODULE.validate_map_files(str(tmp_path / "missing.yaml"))


def test_validate_map_files_rejects_missing_image(tmp_path):
    map_path = write_map(tmp_path, "missing.pgm")

    with pytest.raises(ValueError, match="does not exist or is empty"):
        MODULE.validate_map_files(str(map_path))


def test_validate_map_files_rejects_invalid_image_entry(tmp_path):
    map_path = tmp_path / "map.yaml"
    map_path.write_text("resolution: 0.05\n", encoding="utf-8")

    with pytest.raises(ValueError, match="no valid image entry"):
        MODULE.validate_map_files(str(map_path))


@pytest.mark.parametrize(
    ("now_ns", "stamp_ns", "expected"),
    [
        (10_000_000_000, 9_000_000_000, True),
        (10_000_000_000, 7_000_000_000, False),
        (10_000_000_000, 10_400_000_000, True),
        (10_000_000_000, 10_600_000_000, False),
        (0, 0, False),
    ],
)
def test_transform_freshness(now_ns, stamp_ns, expected):
    assert (
        MODULE.transform_is_fresh(
            now_ns=now_ns,
            stamp_ns=stamp_ns,
            max_age_sec=2.0,
            future_tolerance_sec=0.5,
        )
        is expected
    )
