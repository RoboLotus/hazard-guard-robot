from hazard_guard_thermal_analysis.decision_metadata import (
    effective_threshold_range,
)


def test_effective_threshold_range_preserves_voxel_variation() -> None:
    result = {
        "equipment": [
            {
                "equipment_id": "motor",
                "voxels": [
                    {
                        "trend_analysis": {
                            "effective_adaptive_threshold_c": 55.0
                        }
                    },
                    {
                        "trend_analysis": {
                            "effective_adaptive_threshold_c": 61.5
                        }
                    },
                ],
            }
        ]
    }

    assert effective_threshold_range(result, "motor") == (55.0, 61.5)
    assert effective_threshold_range(result, "missing") == (None, None)
