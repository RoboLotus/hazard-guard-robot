import json

import pytest

from hazard_guard_thermal_analysis.projection import ThermalPoint
from hazard_guard_thermal_analysis.voxel import (
    AnalysisConfig,
    AxisAlignedRoi,
    analyze_points,
    load_config,
    percentile,
)


def test_voxel_statistics_use_p95_and_ambient_delta() -> None:
    config = AnalysisConfig(
        frame_id="equipment_frame",
        voxel_size_m=0.5,
        min_points_per_voxel=3,
        equipment_rois=(
            AxisAlignedRoi("motor", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
        ),
        environment_rois=(
            AxisAlignedRoi("ambient", (2.0, 0.0, 0.0), (3.0, 1.0, 1.0)),
        ),
    )
    points = [
        ThermalPoint(0.1, 0.1, 0.1, temperature, 1.0)
        for temperature in (35.0, 39.0, 46.0)
    ] + [
        ThermalPoint(2.1, 0.1, 0.1, temperature, 1.0)
        for temperature in (24.0, 25.0, 26.0)
    ]

    result = analyze_points(points, config)
    ambient = result["ambient"]
    voxel = result["equipment"][0]["voxels"][0]

    assert ambient["median_temperature_c"] == pytest.approx(25.0)
    assert voxel["voxel_id"] == "motor:0:0:0"
    assert voxel["p95_temperature_c"] == pytest.approx(45.3)
    assert voxel["delta_p95_c"] == pytest.approx(20.3)


def test_unobserved_and_low_count_voxels_are_not_written_as_zero() -> None:
    config = AnalysisConfig(
        frame_id="map",
        voxel_size_m=0.5,
        min_points_per_voxel=2,
        equipment_rois=(
            AxisAlignedRoi("pump", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
        ),
        environment_rois=(),
    )
    result = analyze_points(
        [ThermalPoint(0.1, 0.1, 0.1, 60.0, 1.0)], config
    )

    equipment = result["equipment"][0]
    assert equipment["voxels"] == []
    assert equipment["statistics"] is None
    assert equipment["coverage_ratio"] == 0.0


def test_config_validation_and_percentile(tmp_path) -> None:
    path = tmp_path / "roi.json"
    path.write_text(
        json.dumps(
            {
                "frame_id": "odom",
                "voxel_size_m": 0.1,
                "min_points_per_voxel": 4,
                "equipment_rois": [
                    {"id": "tank", "min": [0, 0, 0], "max": [1, 1, 1]}
                ],
            }
        ),
        encoding="utf-8",
    )

    config = load_config(path)
    assert config.frame_id == "odom"
    assert percentile([0.0, 10.0], 95.0) == pytest.approx(9.5)
