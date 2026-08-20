import unittest

from hazard_guard_dispenser.command_policy import (
    allow_legacy_drop,
    allow_maintenance_command,
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
