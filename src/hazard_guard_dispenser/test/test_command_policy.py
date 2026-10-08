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

    def test_reloaded_cube_can_be_cleared_once_maintenance_is_enabled(self):
        # The gate checked this command but never listed it, so a cube that was
        # picked up and reloaded stayed marked as deployed and was skipped by
        # every later drop.
        self.assertTrue(allow_maintenance_command("reset_installed", True))
        self.assertTrue(
            allow_maintenance_command("reset_installed:B0:B2:1C:4A:13:0A", True)
        )

    def test_clearing_deployment_state_still_needs_maintenance_mode(self):
        self.assertFalse(allow_maintenance_command("reset_installed", False))
        self.assertFalse(
            allow_maintenance_command("reset_installed:B0:B2:1C:4A:13:0A", False)
        )

    def test_unrelated_commands_stay_blocked(self):
        for command in ("drop", "status", "battery", "reset", "reset_installedx"):
            self.assertFalse(allow_maintenance_command(command, True), msg=command)

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
