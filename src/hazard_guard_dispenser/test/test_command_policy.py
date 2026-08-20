import unittest

from hazard_guard_dispenser.command_policy import (
    allow_legacy_drop,
    allow_maintenance_command,
    physical_drop_block_reason,
)


class CommandPolicyTest(unittest.TestCase):
    def test_unkeyed_drop_is_disabled_by_default(self):
        self.assertFalse(allow_legacy_drop(False))
        self.assertTrue(allow_legacy_drop(True))

    def test_manual_servo_commands_are_disabled_by_default(self):
        self.assertFalse(allow_maintenance_command("home", False))
        self.assertFalse(allow_maintenance_command("angle:30", False))
        self.assertTrue(allow_maintenance_command("home", True))
        self.assertTrue(allow_maintenance_command("angle:30", True))

    def test_physical_drop_is_fail_closed(self):
        self.assertEqual(
            physical_drop_block_reason(
                enabled=False, hardware_available=True, armed_count=1
            ),
            "physical_drop_disabled",
        )
        self.assertEqual(
            physical_drop_block_reason(
                enabled=True, hardware_available=False, armed_count=1
            ),
            "hardware_unavailable",
        )
        self.assertEqual(
            physical_drop_block_reason(
                enabled=True,
                hardware_available=True,
                motion_stopped=False,
                armed_count=1,
            ),
            "robot_not_stably_stopped",
        )
        self.assertEqual(
            physical_drop_block_reason(
                enabled=True, hardware_available=True, armed_count=0
            ),
            "no_ble_confirmation_channel",
        )
        self.assertIsNone(
            physical_drop_block_reason(
                enabled=True, hardware_available=True, armed_count=1
            )
        )
