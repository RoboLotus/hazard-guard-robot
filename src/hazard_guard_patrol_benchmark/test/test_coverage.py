from pathlib import Path

import pytest

from hazard_guard_patrol_benchmark.coverage import MapGrid, read_pgm
from hazard_guard_patrol_benchmark.metrics import PoseSample


def _map(tmp_path: Path, rows: list[list[int]], resolution: float = 1.0) -> Path:
    height = len(rows)
    width = len(rows[0])
    pgm = tmp_path / "map.pgm"
    pgm.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii")
        + bytes(value for row in rows for value in row)
    )
    yaml = tmp_path / "map.yaml"
    yaml.write_text(
        "image: map.pgm\n"
        f"resolution: {resolution}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    return yaml


def test_reads_binary_pgm_without_consuming_pixel_whitespace(tmp_path: Path) -> None:
    path = tmp_path / "tiny.pgm"
    path.write_bytes(b"P5\n2 1\n255\n" + bytes([10, 254]))
    width, height, pixels = read_pgm(path)
    assert (width, height) == (2, 1)
    assert pixels == bytes([10, 254])


def test_computes_unique_path_coverage(tmp_path: Path) -> None:
    grid = MapGrid.from_yaml(_map(tmp_path, [[254] * 5 for _ in range(5)]))
    trajectory = [
        PoseSample(0.0, 1.5, 2.5, 0.0),
        PoseSample(1.0, 3.5, 2.5, 0.0),
        PoseSample(2.0, 1.5, 2.5, 0.0),
    ]
    result = grid.coverage(
        trajectory,
        start_x=1.5,
        start_y=2.5,
        inspection_radius_m=0.49,
    )
    assert result.patrolable_cell_count == 25
    assert result.observed_cell_count == 3
    assert result.coverage_percent == pytest.approx(12.0)


def test_excludes_disconnected_free_region(tmp_path: Path) -> None:
    rows = [
        [254, 254, 0, 254, 254],
        [254, 254, 0, 254, 254],
        [254, 254, 0, 254, 254],
    ]
    grid = MapGrid.from_yaml(_map(tmp_path, rows))
    reachable = grid.reachable_mask(0.5, 0.5, clearance_m=0.0)
    assert sum(reachable) == 6


def test_clearance_respects_map_boundary(tmp_path: Path) -> None:
    grid = MapGrid.from_yaml(_map(tmp_path, [[254] * 5 for _ in range(5)]))
    reachable = grid.reachable_mask(2.5, 2.5, clearance_m=1.0)
    assert sum(reachable) == 9
