import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from hazard_guard_dispenser.request_ledger import (
    IdempotencyConflictError,
    RequestLedger,
    RequestLedgerError,
)


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

    def test_request_id_reuse_with_different_detection_is_rejected(self):
        ledger = RequestLedger(self.path)
        ledger.claim(request_id="req-1", detection_id="thermal-1")

        with self.assertRaises(IdempotencyConflictError):
            ledger.claim(request_id="req-1", detection_id="thermal-2")

    def test_busy_request_can_retry_once_without_reusing_actuation(self):
        ledger = RequestLedger(self.path)
        ledger.claim(request_id="req-1", detection_id="thermal-1")
        ledger.transition(
            "req-1", "rejected_busy", actuation_started=False
        )

        record, created = ledger.claim(
            request_id="req-1", detection_id="thermal-1"
        )

        self.assertTrue(created)
        self.assertEqual(record["state"], "accepted")

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

    def test_corrupt_sqlite_ledger_blocks_a_physical_request(self):
        self.path.write_text("not a sqlite database", encoding="utf-8")

        with self.assertRaises(RequestLedgerError):
            RequestLedger(self.path)

    def test_two_node_connections_claim_only_once(self):
        first = RequestLedger(self.path)
        second = RequestLedger(self.path)
        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(
                executor.map(
                    lambda ledger: ledger.claim(
                        request_id="req-1", detection_id="thermal-1"
                    ),
                    (first, second),
                )
            )
        self.assertEqual(sum(created for _, created in results), 1)
