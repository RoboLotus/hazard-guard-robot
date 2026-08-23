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
        link._battery[address] = (
            11.4,
            time.monotonic() - 10.0,
            time.time() - 10.0,
        )

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

    def test_structured_snapshot_preserves_identity_and_freshness(self):
        link = CubeLink()
        address = "11:22:33:44:55:66"
        link._clients[address] = _Client(connected=True)
        link._on_battery(address, bytes([126]))

        snapshot = link.battery_snapshot(stale_after=60.0)

        self.assertEqual(len(snapshot), 1)
        self.assertEqual(snapshot[0]["address"], address)
        self.assertEqual(snapshot[0]["voltage"], 12.6)
        self.assertEqual(snapshot[0]["percent"], 100)
        self.assertTrue(snapshot[0]["connected"])
        self.assertFalse(snapshot[0]["stale"])
        self.assertGreater(snapshot[0]["updated_at_unix_ms"], 0)

    def test_connected_legacy_cube_is_visible_without_battery_reading(self):
        link = CubeLink()
        address = "AA:00:00:00:00:01"
        link._clients[address] = _Client(connected=True)

        snapshot = link.status_snapshot(stale_after=60.0)

        self.assertEqual(snapshot["connected"], 1)
        self.assertEqual(len(snapshot["beacons"]), 1)
        record = snapshot["beacons"][0]
        self.assertEqual(record["address"], address)
        self.assertTrue(record["connected"])
        self.assertFalse(record["battery_supported"])
        self.assertIsNone(record["voltage"])
        self.assertIsNone(record["updated_at_unix_ms"])


if __name__ == "__main__":
    unittest.main()
