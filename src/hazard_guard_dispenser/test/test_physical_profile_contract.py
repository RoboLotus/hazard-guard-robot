import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class PhysicalProfileContractTests(unittest.TestCase):
    def test_profile_keeps_physical_actuation_disabled(self):
        profile = (PACKAGE_ROOT / "config" / "dispenser_physical.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("angle_dump: 60", profile)
        self.assertIn("servo_max_angle: 75", profile)
        self.assertIn("enable_physical_drop: false", profile)

    def test_launch_loads_packaged_physical_profile(self):
        launch = (PACKAGE_ROOT / "launch" / "dispenser.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"dispenser_physical.yaml"', launch)
        self.assertIn('default_value="false"', launch)


if __name__ == "__main__":
    unittest.main()
