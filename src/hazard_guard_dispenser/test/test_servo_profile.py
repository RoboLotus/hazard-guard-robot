import unittest

from hazard_guard_dispenser.servo_profile import ServoProfile


class ServoProfileTests(unittest.TestCase):
    def test_physical_profile_accepts_measured_dump_angle(self):
        profile = ServoProfile(
            home_angle=0,
            dump_angle=60,
            minimum_angle=0,
            maximum_angle=75,
            step_deg=3,
            step_delay_sec=0.03,
        )
        self.assertEqual(profile.validate_target(60), 60)

    def test_rejects_target_outside_mechanical_range(self):
        profile = ServoProfile(0, 60, 0, 75, 3, 0.03)
        with self.assertRaises(ValueError):
            profile.validate_target(90)

    def test_rejects_invalid_motion_configuration(self):
        with self.assertRaises(ValueError):
            ServoProfile(0, 60, 0, 75, 0, 0.03)
        with self.assertRaises(ValueError):
            ServoProfile(0, 0, 0, 75, 3, 0.03)


if __name__ == "__main__":
    unittest.main()
