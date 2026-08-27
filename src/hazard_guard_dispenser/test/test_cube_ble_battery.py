import asyncio
import tempfile
import time
import unittest
from pathlib import Path

from hazard_guard_dispenser.cube_ble import CubeLink


class _Client:
    def __init__(self, connected=True):
        self.is_connected = connected
        self.address = "fake"


class _BatteryClient(_Client):
    def __init__(self, value, connected=True):
        super().__init__(connected=connected)
        self.value = value
        self.reads = 0

    async def read_gatt_char(self, _uuid):
        self.reads += 1
        return bytes([self.value])


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

    def test_periodic_refresh_re_reads_connected_cube_battery(self):
        link = CubeLink()
        address = "AA:BB:CC:DD:EE:01"
        client = _BatteryClient(118)
        link._clients[address] = client

        asyncio.run(link._refresh_batteries())

        record = link.battery_snapshot(stale_after=1.0)[0]
        self.assertEqual(client.reads, 1)
        self.assertEqual(record["voltage"], 11.8)
        self.assertFalse(record["stale"])

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

    def test_low_battery_report_marks_cube_unavailable(self):
        link = CubeLink()
        address = "AA:00:00:00:00:02"
        link._clients[address] = _Client(connected=True)
        link._on_report(address, bytes([4]))

        record = link.status_snapshot()["beacons"][0]
        self.assertTrue(record["reported_unavailable"])
        self.assertFalse(link.arm_is_valid())

    def test_dropped_cube_is_persisted_and_excluded_after_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "installed.json"
            address = "AA:00:00:00:00:04"
            link = CubeLink(installed_state_path=state_path)
            link.mark_installed(address)

            restored = CubeLink(installed_state_path=state_path)
            restored._clients[address] = _Client(connected=True)
            record = restored.status_snapshot()["beacons"][0]

            self.assertTrue(record["installed"])
            self.assertTrue(record["reported_unavailable"])
            self.assertIn(address, restored._installed)
            restored.reset_installed(address)
            self.assertNotIn(address, CubeLink(
                installed_state_path=state_path
            )._installed)

    def test_corrupt_installed_ledger_blocks_arming(self):
        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "installed.json"
            state_path.write_text("not-json", encoding="utf-8")
            link = CubeLink(installed_state_path=state_path)
            address = "AA:00:00:00:00:05"
            link._clients[address] = _Client(connected=True)

            self.assertFalse(link.installed_persistence_ok())
            self.assertEqual(asyncio.run(link._send_all(b"A", 1, 0)), 0)

    def test_cancel_is_sent_even_to_unavailable_cube(self):
        link = CubeLink()
        address = "AA:00:00:00:00:03"
        client = _Client(connected=True)
        client.address = address
        link._clients[address] = client
        link._unavailable.add(address)
        writes = []

        async def write_one(target, payload):
            writes.append((target.address, payload))
            return True

        link._write_one = write_one
        sent = asyncio.run(link._send_all(b"C", 1, 0))

        self.assertEqual(sent, 1)
        self.assertEqual(writes, [(address, b"C")])

    def test_actuation_claim_and_invalidation_are_mutually_exclusive(self):
        link = CubeLink()
        link._arm_state = "armed"
        self.assertTrue(link.begin_actuation())
        self.assertFalse(link.begin_actuation())

        blocked = CubeLink()
        blocked._arm_state = "armed"
        blocked.cancel_all()
        self.assertFalse(blocked.begin_actuation())


if __name__ == "__main__":
    unittest.main()
