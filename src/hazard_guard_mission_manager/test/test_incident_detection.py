import unittest

from hazard_guard_mission_manager.incident_detection import (
    thermal_observations,
    thermal_visit_index,
)


class IncidentDetectionTests(unittest.TestCase):
    def test_builds_stable_incident_from_completed_visit(self):
        payload = {
            "frame_id": "map",
            "trend_analysis": {"visit_index": 7},
            "equipment": [
                {
                    "equipment_id": "pump-01",
                    "trend_status": "critical",
                    "voxels": [
                        {
                            "center": [1.0, 2.0, 0.5],
                            "max_temperature_c": 83.4,
                            "trend_analysis": {"status": "critical"},
                        }
                    ],
                }
            ],
        }
        result = thermal_observations(payload, mission_id="mission-1")
        self.assertEqual(len(result), 1)
        self.assertEqual(
            result[0]["incident_id"],
            "thermal:mission-1:pump-01:visit-7",
        )
        self.assertEqual(result[0]["severity"], "critical")
        self.assertEqual(result[0]["mission_id"], "mission-1")
        self.assertTrue(result[0]["detection_id"].startswith("thermal:"))
        replay = thermal_observations(payload, mission_id="mission-1")
        other_visit = thermal_observations(
            {**payload, "trend_analysis": {"visit_index": 8}},
            mission_id="mission-1",
        )
        self.assertEqual(result[0]["detection_id"], replay[0]["detection_id"])
        self.assertNotEqual(
            result[0]["detection_id"],
            other_visit[0]["detection_id"],
        )
        self.assertEqual(result[0]["x"], 1.0)

    def test_reports_normal_for_monitoring_release_without_opening_policy(self):
        payload = {
            "trend_analysis": {"visit_index": 8},
            "equipment": [
                {"equipment_id": "pump-01", "trend_status": "normal"}
            ],
        }
        result = thermal_observations(payload, mission_id="mission-1")
        self.assertEqual(result[0]["severity"], "normal")

    def test_rejects_missing_visit_index(self):
        with self.assertRaises(ValueError):
            thermal_visit_index({"trend_analysis": {}})

    def test_rejects_malformed_equipment_and_voxels(self):
        with self.assertRaises(ValueError):
            thermal_observations(
                {"trend_analysis": {"visit_index": 1}, "equipment": None},
                mission_id="mission-1",
            )
        with self.assertRaises(ValueError):
            thermal_observations(
                {
                    "trend_analysis": {"visit_index": 1},
                    "equipment": [
                        {
                            "equipment_id": "pump-01",
                            "voxels": None,
                        }
                    ],
                },
                mission_id="mission-1",
            )

    def test_rejects_non_finite_temperature(self):
        with self.assertRaises(ValueError):
            thermal_observations(
                {
                    "trend_analysis": {"visit_index": 1},
                    "equipment": [
                        {
                            "equipment_id": "pump-01",
                            "voxels": [
                                {
                                    "center": [0, 0, 0],
                                    "max_temperature_c": "NaN",
                                }
                            ],
                        }
                    ],
                },
                mission_id="mission-1",
            )


if __name__ == "__main__":
    unittest.main()
