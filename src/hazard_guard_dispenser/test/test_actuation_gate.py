import threading
import unittest
from concurrent.futures import ThreadPoolExecutor

from hazard_guard_dispenser.actuation_gate import ActuationGate


class ActuationGateTest(unittest.TestCase):
    def test_cancel_before_claim_prevents_actuation(self):
        gate = ActuationGate()
        marked = []

        self.assertTrue(gate.request_cancel("req-1"))
        self.assertFalse(
            gate.claim_actuation("req-1", mark_started=lambda: marked.append(True))
        )
        self.assertEqual(marked, [])

    def test_cancel_waiting_on_claim_is_rejected_after_durable_start(self):
        gate = ActuationGate()
        marking = threading.Event()
        release = threading.Event()

        def mark_started():
            marking.set()
            release.wait(timeout=2.0)

        with ThreadPoolExecutor(max_workers=2) as executor:
            claim = executor.submit(
                gate.claim_actuation,
                "req-1",
                mark_started=mark_started,
            )
            self.assertTrue(marking.wait(timeout=1.0))
            cancel = executor.submit(gate.request_cancel, "req-1")
            self.assertFalse(cancel.done())
            release.set()
            self.assertTrue(claim.result(timeout=1.0))
            self.assertFalse(cancel.result(timeout=1.0))


if __name__ == "__main__":
    unittest.main()
