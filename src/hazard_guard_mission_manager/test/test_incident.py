import unittest

from hazard_guard_mission_manager.incident import (
    DECISION_COMPLETE_MONITORING,
    DECISION_DROP_THEN_MONITOR,
    IncidentApprovalLatch,
    IncidentConflictError,
)


class IncidentApprovalLatchTests(unittest.TestCase):
    def test_open_is_idempotent_for_same_incident(self):
        latch = IncidentApprovalLatch()
        first, created = latch.open({"incident_id": "thermal-pump"})
        replay, replay_created = latch.open({"incident_id": "thermal-pump"})
        self.assertTrue(created)
        self.assertFalse(replay_created)
        self.assertEqual(first, replay)

    def test_monitoring_requires_second_explicit_decision(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        latch.transition("monitoring")
        self.assertTrue(latch.is_paused())
        record, created = latch.decide(
            incident_id="thermal-pump",
            request_id="decision-2",
            decision=DECISION_COMPLETE_MONITORING,
            operator_id="operator",
        )
        self.assertTrue(created)
        self.assertEqual(record["state"], "resuming")

    def test_request_id_reuse_with_different_decision_is_rejected(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        with self.assertRaises(IncidentConflictError):
            latch.decide(
                incident_id="thermal-pump",
                request_id="decision-1",
                decision=DECISION_COMPLETE_MONITORING,
                operator_id="operator",
            )


if __name__ == "__main__":
    unittest.main()
