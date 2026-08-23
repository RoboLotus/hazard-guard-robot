import time
import unittest

from hazard_guard_dispenser.cube_ble import CubeLink


class _Client:
    def __init__(self, connected=True):
        self.is_connected = connected


class CubeBatteryTests(unittest.TestCase):
    def test_battery_snapshot_marks_disconnect_and_stale(self):
        link = CubeLink()
        address = "AA:BB:CC:DD:EE:FF"
        link._clients[address] = _Client(connected=False)
        link._battery[address] = (11.4, time.monotonic() - 10.0)

        voltage, percent, connected, stale = link.battery_levels(
            stale_after=1.0
        )[address]

        self.assertEqual(voltage, 11.4)
        self.assertTrue(0 <= percent <= 100)
        self.assertFalse(connected)
        self.assertTrue(stale)

    def test_battery_callback_replaces_reading_atomically(self):
        link = CubeLink()
        address = "AA:BB:CC:DD:EE:FF"
        link._clients[address] = _Client(connected=True)

        link._on_battery(address, bytes([121]))

        voltage, percent, connected, stale = link.battery_levels(
            stale_after=60.0
        )[address]
        self.assertEqual(voltage, 12.1)
        self.assertEqual(percent, 86)
        self.assertTrue(connected)
        self.assertFalse(stale)


if __name__ == "__main__":
    unittest.main()
