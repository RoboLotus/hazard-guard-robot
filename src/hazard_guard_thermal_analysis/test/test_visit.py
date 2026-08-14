from hazard_guard_thermal_analysis.visit import PatrolVisitAccumulator


def equipment(equipment_id, temperature, *, point_count=10):
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
        "thresholds": {
            "warning_temperature_c": 45.0,
            "critical_temperature_c": 80.0,
            "warning_delta_c": 15.0,
        },
        "voxels": [
            {
                "voxel_id": f"{equipment_id}:0:0:0",
                "index": [0, 0, 0],
                "center": [0.1, 0.1, 0.1],
                "mean_temperature_c": temperature - 2,
                "median_temperature_c": temperature - 1,
                "p90_temperature_c": temperature - 0.5,
                "p95_temperature_c": temperature,
                "max_temperature_c": temperature + 1,
                "point_count": point_count,
                "valid": True,
            }
        ],
    }


def frame(ambient, *equipment_items):
    return {
        "frame_id": "map",
        "ambient": {
            "mean_temperature_c": ambient,
            "median_temperature_c": ambient,
            "point_count": 20,
        },
        "equipment": list(equipment_items),
    }


def test_collects_only_the_focused_equipment():
    visit = PatrolVisitAccumulator()
    visit.start()
    visit.focus("motor")
    visit.add(frame(25, equipment("motor", 40), equipment("pump", 70)))

    result = visit.finalize()

    assert visit.equipment_ids == ("motor",)
    assert [item["equipment_id"] for item in result["equipment"]] == ["motor"]


def test_one_lap_uses_temporal_medians_instead_of_a_single_frame_peak():
    visit = PatrolVisitAccumulator()
    visit.start()
    visit.focus("motor")
    visit.add(frame(25, equipment("motor", 40, point_count=8)))
    visit.add(frame(26, equipment("motor", 44, point_count=10)))
    visit.add(frame(100, equipment("motor", 100, point_count=100)))

    result = visit.finalize({"sec": 10, "nanosec": 20})
    motor = result["equipment"][0]
    voxel = motor["voxels"][0]

    assert result["ambient"]["median_temperature_c"] == 26
    assert voxel["p95_temperature_c"] == 44
    assert voxel["delta_p95_c"] == 18
    assert voxel["frame_count"] == 3
    assert voxel["point_count"] == 10
    assert motor["statistics"]["p95_temperature_c"] == 44
    assert motor["capture_frame_count"] == 3
    assert result["visit_capture"]["equipment_count"] == 1
    assert result["stamp"] == {"sec": 10, "nanosec": 20}


def test_inactive_accumulator_does_not_create_history_data():
    visit = PatrolVisitAccumulator()
    visit.add(frame(25, equipment("motor", 50)))

    assert visit.equipment_ids == ()
    assert visit.finalize()["equipment"] == []