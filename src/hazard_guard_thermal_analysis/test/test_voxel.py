import json
from pathlib import Path

import pytest

from hazard_guard_thermal_analysis.projection import ThermalPoint
from hazard_guard_thermal_analysis.voxel import AnalysisConfig, AxisAlignedRoi, analyze_points, load_config, percentile


def test_schema1_keeps_legacy_ambient_delta_compatibility() -> None:
    config = AnalysisConfig(
        frame_id="equipment_frame", voxel_size_m=0.5, min_points_per_voxel=3,
        equipment_rois=(AxisAlignedRoi("motor", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),),
        environment_rois=(AxisAlignedRoi("ambient", (2.0, 0.0, 0.0), (3.0, 1.0, 1.0)),),
    )
    points = [ThermalPoint(0.1, 0.1, 0.1, temperature, 1.0) for temperature in (35.0, 39.0, 46.0)]
    points += [ThermalPoint(2.1, 0.1, 0.1, temperature, 1.0) for temperature in (24.0, 25.0, 26.0)]
    result = analyze_points(points, config)
    voxel = result["equipment"][0]["voxels"][0]
    assert result["ambient"]["median_temperature_c"] == pytest.approx(25.0)
    assert voxel["p95_temperature_c"] == pytest.approx(45.3)
    assert voxel["delta_p95_c"] == pytest.approx(20.3)


def test_schema2_uses_same_material_reference_not_floor_for_decision_delta() -> None:
    equipment = AxisAlignedRoi(
        "waste", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0),
        critical_temperature_c=65.0, reference_roi_id="reference",
    )
    config = AnalysisConfig(
        frame_id="odom", voxel_size_m=0.5, min_points_per_voxel=3,
        equipment_rois=(equipment,),
        environment_rois=(AxisAlignedRoi("floor", (2.0, 0.0, 0.0), (3.0, 1.0, 1.0)),),
        reference_rois=(AxisAlignedRoi("reference", (4.0, 0.0, 0.0), (5.0, 1.0, 1.0)),),
        schema_version=2, min_points_per_roi_for_p95=3,
        recommended_points_per_roi_for_p95=5,
    )
    points = [ThermalPoint(0.1, 0.1, 0.1, value, 1.0, i, 0) for i, value in enumerate((45.0, 46.0, 47.0))]
    points += [ThermalPoint(2.1, 0.1, 0.1, value, 1.0) for value in (20.0, 21.0, 22.0)]
    points += [ThermalPoint(4.1, 0.1, 0.1, value, 1.0) for value in (30.0, 31.0, 32.0)]
    result = analyze_points(points, config, simulated=False)
    voxel = result["equipment"][0]["voxels"][0]
    assert voxel["ambient_delta_p95_c"] == pytest.approx(25.9)
    assert voxel["reference_delta_p95_c"] == pytest.approx(15.9)
    assert voxel["delta_p95_c"] == pytest.approx(15.9)
    assert result["equipment"][0]["p95_valid"] is True
    assert "below_recommended_p95_samples" in result["equipment"][0]["quality_flags"]


def test_radiometric_three_by_three_cluster_is_counted() -> None:
    roi = AxisAlignedRoi(
        "motor", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0),
        simulation_critical_temperature_c=90.0,
    )
    config = AnalysisConfig(
        frame_id="odom", voxel_size_m=1.0, min_points_per_voxel=5,
        equipment_rois=(roi,), environment_rois=(), schema_version=2,
        min_points_per_roi_for_p95=5, recommended_points_per_roi_for_p95=9,
        min_hot_cluster_pixels=9, min_adjacent_hot_voxels=2,
    )
    points = [
        ThermalPoint(0.1, 0.1, 0.1, 95.0, 1.0, u, v)
        for v in range(3) for u in range(3)
    ]
    voxel = analyze_points(points, config, simulated=True)["equipment"][0]["voxels"][0]
    assert voxel["radiometric_pixel_count"] == 9
    assert voxel["max_hot_cluster_pixels"] == 9


def test_unobserved_and_low_count_voxels_are_not_written_as_zero() -> None:
    config = AnalysisConfig(
        frame_id="map", voxel_size_m=0.5, min_points_per_voxel=2,
        equipment_rois=(AxisAlignedRoi("pump", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),),
        environment_rois=(),
    )
    result = analyze_points([ThermalPoint(0.1, 0.1, 0.1, 60.0, 1.0)], config)
    equipment = result["equipment"][0]
    assert equipment["voxels"] == []
    assert equipment["statistics"] is None
    assert equipment["coverage_ratio"] == 0.0


def test_config_validation_and_percentile(tmp_path) -> None:
    path = tmp_path / "roi.json"
    path.write_text(json.dumps({
        "frame_id": "odom", "voxel_size_m": 0.1, "min_points_per_voxel": 4,
        "equipment_rois": [{"id": "tank", "min": [0, 0, 0], "max": [1, 1, 1]}],
    }), encoding="utf-8")
    config = load_config(path)
    assert config.frame_id == "odom"
    assert percentile([0.0, 10.0], 95.0) == pytest.approx(9.5)


def test_facility_schema2_loads_reviewed_thresholds_and_quality_rules() -> None:
    path = Path(__file__).parents[1] / "config" / "demo_facility_scaled_rois.json"
    config = load_config(path)
    rois = {roi.roi_id: roi for roi in config.equipment_rois}
    waste = rois["bunker_waste_pile"]
    motor = rois["primary_shredder_motor"]
    pump = rois["secondary_processor_pump"]
    tank = rois["baler_hydraulic_tank"]
    assert config.schema_version == 2
    assert config.min_points_per_voxel == 5
    assert config.min_points_per_roi_for_p95 == 40
    assert config.recommended_points_per_roi_for_p95 == 100
    assert (config.min_hot_cluster_pixels, config.min_adjacent_hot_voxels) == (9, 2)
    assert (waste.watch_temperature_c, waste.warning_temperature_c, waste.critical_temperature_c) == (40.0, 50.0, 65.0)
    assert waste.reference_roi_id == "bunker_waste_reference"
    assert (motor.simulation_watch_temperature_c, motor.simulation_warning_temperature_c, motor.simulation_critical_temperature_c) == (70.0, 80.0, 90.0)
    assert (motor.baseline_watch_delta_c, motor.baseline_warning_delta_c, motor.baseline_critical_delta_c) == (10.0, 15.0, 20.0)
    assert (pump.air_watch_delta_c, pump.air_warning_delta_c, pump.air_critical_delta_c) == (30.0, 35.0, 40.0)
    assert tank.surface_alone_can_trip is False
    assert (tank.oil_watch_temperature_c, tank.oil_warning_temperature_c, tank.oil_critical_temperature_c) == (50.0, 60.0, 70.0)


def test_config_rejects_non_increasing_threshold_levels(tmp_path) -> None:
    path = tmp_path / "invalid_thresholds.json"
    path.write_text(json.dumps({"equipment_rois": [{
        "id": "motor", "min": [0, 0, 0], "max": [1, 1, 1],
        "watch_temperature_c": 70.0, "warning_temperature_c": 65.0,
        "critical_temperature_c": 85.0,
    }]}), encoding="utf-8")
    with pytest.raises(ValueError, match="thresholds must increase"):
        load_config(path)
