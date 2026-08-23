import unittest

from hazard_guard_dispenser.person_safety import PersonSafetyLatch


class PersonSafetyLatchTests(unittest.TestCase):
    def test_unknown_and_stale_state_fail_closed(self):
        latch = PersonSafetyLatch(required=True, timeout_sec=1.0)
        self.assertEqual(latch.block_reason(now_monotonic=10.0), "person_safety_unknown")
        latch.update(state=0, detector_stale=False, now_monotonic=10.0)
        self.assertIsNone(latch.block_reason(now_monotonic=10.9))
        self.assertEqual(latch.block_reason(now_monotonic=11.1), "person_safety_stale")

    def test_stop_and_detector_stale_fail_closed(self):
        latch = PersonSafetyLatch(required=True, timeout_sec=1.0)
        latch.update(state=3, detector_stale=False, now_monotonic=10.0)
        self.assertEqual(latch.block_reason(now_monotonic=10.1), "person_safety_not_clear")
        latch.update(state=0, detector_stale=True, now_monotonic=10.2)
        self.assertEqual(latch.block_reason(now_monotonic=10.3), "person_detector_stale")

    def test_optional_supervision_allows_bench_mode(self):
        latch = PersonSafetyLatch(required=False, timeout_sec=1.0)
        self.assertIsNone(latch.block_reason(now_monotonic=10.0))


if __name__ == "__main__":
    unittest.main()
