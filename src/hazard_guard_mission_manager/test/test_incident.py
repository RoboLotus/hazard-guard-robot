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
        latch.mark_dispense_result("monitoring")
        self.assertTrue(latch.is_paused())
        record, created = latch.decide(
            incident_id="thermal-pump",
            request_id="decision-2",
            decision=DECISION_COMPLETE_MONITORING,
            operator_id="operator",
        )
        self.assertTrue(created)
        self.assertEqual(record["state"], "resuming")
        self.assertEqual(record["decision"], DECISION_DROP_THEN_MONITOR)
        self.assertEqual(len(record["decision_history"]), 2)

    def test_monitoring_cannot_resolve_without_release_decision(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        latch.mark_dispense_result("monitoring")
        with self.assertRaises(IncidentConflictError):
            latch.resolve_resume()
        latch.mark_monitoring_normalized()
        self.assertTrue(latch.is_paused())

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

    def test_old_replay_returns_old_incident_snapshot(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "incident-a"})
        first, _ = latch.decide(
            incident_id="incident-a",
            request_id="decision-a",
            decision="resume",
            operator_id="operator",
        )
        latch.resolve_resume()
        latch.open({"incident_id": "incident-b"})

        replay, created = latch.decide(
            incident_id="incident-a",
            request_id="decision-a",
            decision="resume",
            operator_id="operator",
        )
        self.assertFalse(created)
        self.assertEqual(replay["incident_id"], "incident-a")
        self.assertEqual(replay["state"], first["state"])

    def test_operator_id_is_required(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        with self.assertRaises(ValueError):
            latch.decide(
                incident_id="thermal-pump",
                request_id="decision-1",
                decision="resume",
                operator_id="   ",
            )


if __name__ == "__main__":
    unittest.main()
