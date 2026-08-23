import unittest

from hazard_guard_dispenser.battery_policy import BatteryPolicy


class BatteryPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = BatteryPolicy()

    def test_classifies_operational_ranges(self):
        self.assertEqual(self.policy.state(12.0), "normal")
        self.assertEqual(self.policy.state(10.5), "low")
        self.assertEqual(self.policy.state(10.0), "critical")
        self.assertEqual(self.policy.state(20.0), "invalid")
        self.assertEqual(self.policy.state(12.0, stale=True), "unknown")

    def test_only_fresh_critical_or_invalid_readings_block_connected_cube(self):
        self.assertFalse(
            self.policy.available_for_drop(10.0, connected=True)
        )
        self.assertFalse(
            self.policy.available_for_drop(20.0, connected=True)
        )
        self.assertFalse(self.policy.available_for_drop(None, connected=True))
        self.assertTrue(
            self.policy.available_for_drop(
                None, connected=True, allow_unknown=True
            )
        )
        self.assertFalse(
            self.policy.available_for_drop(10.0, connected=True, stale=True)
        )
        self.assertFalse(
            self.policy.available_for_drop(12.0, connected=False)
        )

    def test_rejects_invalid_threshold_order(self):
        with self.assertRaises(ValueError):
            BatteryPolicy(low_voltage=9.5, critical_voltage=10.0)
        with self.assertRaises(ValueError):
            BatteryPolicy(low_voltage=float("nan"))


if __name__ == "__main__":
    unittest.main()
