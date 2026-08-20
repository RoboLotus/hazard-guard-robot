import tempfile
import unittest
from pathlib import Path

from hazard_guard_dispenser.request_ledger import RequestLedger


class RequestLedgerTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "requests.json"

    def tearDown(self):
        self.directory.cleanup()

    def test_same_request_is_never_claimed_twice(self):
        ledger = RequestLedger(self.path)

        first, created = ledger.claim(request_id="req-1", detection_id="thermal-1")
        replay, replay_created = ledger.claim(
            request_id="req-1", detection_id="thermal-1"
        )

        self.assertTrue(created)
        self.assertFalse(replay_created)
        self.assertEqual(replay, first)

    def test_detection_id_blocks_a_retry_with_a_new_request_id(self):
        ledger = RequestLedger(self.path)
        ledger.claim(request_id="req-1", detection_id="thermal-1")

        record, created = ledger.claim(request_id="req-2", detection_id="thermal-1")

        self.assertFalse(created)
        self.assertEqual(record["request_id"], "req-1")

    def test_restart_restores_terminal_result_without_replaying(self):
        first = RequestLedger(self.path)
        first.claim(request_id="req-1", detection_id="thermal-1")
        first.transition("req-1", "succeeded", result_detail="ble_drop_confirmed")

        restarted = RequestLedger(self.path)
        record, created = restarted.claim(request_id="req-1", detection_id="thermal-1")

        self.assertFalse(created)
        self.assertEqual(record["state"], "succeeded")
        self.assertEqual(record["result_detail"], "ble_drop_confirmed")

    def test_restart_marks_an_interrupted_command_for_manual_recovery(self):
        first = RequestLedger(self.path)
        first.claim(request_id="req-1", detection_id="thermal-1")
        first.transition("req-1", "dispensing")

        restarted = RequestLedger(self.path)
        record, created = restarted.claim(request_id="req-1", detection_id="thermal-1")

        self.assertFalse(created)
        self.assertEqual(record["state"], "recovery_required")
