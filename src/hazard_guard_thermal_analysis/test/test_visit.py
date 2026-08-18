from hazard_guard_thermal_analysis.visit import PatrolVisitAccumulator


def equipment(equipment_id, temperature, *, point_count=50, reference_delta=None):
    return {
        "equipment_id": equipment_id,
        "observed_voxel_count": 1,
        "configured_voxel_count": 4,
        "coverage_ratio": 0.25,
        "statistics": {
            "mean_temperature_c": temperature - 2,
            "median_temperature_c": temperature - 1,
            "p90_temperature_c": temperature - 0.5,
            "p95_temperature_c": temperature,
            "max_temperature_c": temperature + 1,
            "point_count": point_count,
        },
        "p95_valid": True,
        "quality_flags": [],
        "thresholds": {
            "watch_temperature_c": 45.0,
            "critical_temperature_c": 80.0,
            "reference_roi_id": "material_reference" if reference_delta is not None else None,
        },
        "voxels": [{
            "voxel_id": f"{equipment_id}:0:0:0",
            "index": [0, 0, 0],
            "center": [0.1, 0.1, 0.1],
            "mean_temperature_c": temperature - 2,
            "median_temperature_c": temperature - 1,
            "p90_temperature_c": temperature - 0.5,
            "p95_temperature_c": temperature,
            "max_temperature_c": temperature + 1,
            "point_count": point_count,
            "reference_delta_p95_c": reference_delta,
            "radiometric_pixel_count": 12,
            "max_hot_cluster_pixels": 9,
            "valid": True,
        }],
    }


def frame(ambient, *equipment_items, reference=None):
    result = {
        "schema_version": 2,
        "frame_id": "odom",
        "quality": {
            "min_points_per_roi_for_p95": 40,
            "recommended_points_per_roi_for_p95": 100,
        },
        "ambient": {
            "mean_temperature_c": ambient,
            "median_temperature_c": ambient,
            "point_count": 20,
        },
        "references": {},
        "equipment": list(equipment_items),
    }
    if reference is not None:
        result["references"]["material_reference"] = {
            "mean_temperature_c": reference,
            "median_temperature_c": reference,
            "point_count": 20,
        }
    return result


def test_collects_only_the_focused_equipment():
    visit = PatrolVisitAccumulator()
    visit.start()
    visit.focus("motor")
    visit.add(frame(25, equipment("motor", 40), equipment("pump", 70)))
    result = visit.finalize()
    assert visit.equipment_ids == ("motor",)
    assert [item["equipment_id"] for item in result["equipment"]] == ["motor"]


def test_one_lap_uses_temporal_medians_without_universal_floor_correction():
    visit = PatrolVisitAccumulator()
    visit.start()
    visit.focus("motor")
    visit.add(frame(25, equipment("motor", 40, point_count=48)))
    visit.add(frame(26, equipment("motor", 44, point_count=50)))
    visit.add(frame(100, equipment("motor", 100, point_count=500)))
    result = visit.finalize({"sec": 10, "nanosec": 20})
    motor = result["equipment"][0]
    voxel = motor["voxels"][0]
    assert result["ambient"]["median_temperature_c"] == 26
    assert voxel["p95_temperature_c"] == 44
    assert voxel["delta_p95_c"] is None
    assert voxel["frame_count"] == 3
    assert voxel["point_count"] == 50
    assert motor["statistics"]["p95_temperature_c"] == 44
    assert motor["p95_valid"] is True
    assert "below_recommended_p95_samples" in motor["quality_flags"]
    assert motor["capture_frame_count"] == 3
    assert result["schema_version"] == 2
    assert result["frame_id"] == "odom"
    assert result["stamp"] == {"sec": 10, "nanosec": 20}


def test_same_material_reference_is_preserved_across_lap():
    visit = PatrolVisitAccumulator()
    visit.start()
    visit.focus("waste")
    visit.add(frame(25, equipment("waste", 50, reference_delta=20), reference=30))
    visit.add(frame(25, equipment("waste", 52, reference_delta=21), reference=31))
    result = visit.finalize()
    waste = result["equipment"][0]
    assert result["references"]["material_reference"]["median_temperature_c"] == 30.5
    assert waste["reference_delta_p95_c"] == 20.5
    assert waste["voxels"][0]["reference_delta_p95_c"] == 20.5


def test_inactive_accumulator_does_not_create_history_data():
    visit = PatrolVisitAccumulator()
    visit.add(frame(25, equipment("motor", 50)))
    assert visit.equipment_ids == ()
    assert visit.finalize()["equipment"] == []
