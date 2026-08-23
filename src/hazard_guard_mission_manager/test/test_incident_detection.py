import unittest

from hazard_guard_mission_manager.incident_detection import thermal_observations


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
        self.assertEqual(result[0]["incident_id"], "thermal:pump-01:visit-7")
        self.assertEqual(result[0]["severity"], "critical")
        self.assertEqual(result[0]["mission_id"], "mission-1")
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


if __name__ == "__main__":
    unittest.main()
