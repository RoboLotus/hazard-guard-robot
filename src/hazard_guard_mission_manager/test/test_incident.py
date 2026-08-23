import unittest

from hazard_guard_mission_manager.incident import (
    DECISION_ACKNOWLEDGE_FIELD_CHECK,
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
        latch.mark_dispense_succeeded(dispenser_request_id="drop-1")
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
        latch.mark_dispense_succeeded(dispenser_request_id="drop-1")
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

    def test_request_id_must_match_dispenser_contract(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        with self.assertRaises(ValueError):
            latch.decide(
                incident_id="thermal-pump",
                request_id="invalid request id",
                decision="resume",
                operator_id="operator",
            )

    def test_dispense_success_follows_original_decision(self):
        monitor = IncidentApprovalLatch()
        monitor.open({"incident_id": "monitor"})
        monitor.decide(
            incident_id="monitor",
            request_id="decision-monitor",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        self.assertEqual(
            monitor.mark_dispense_succeeded(
                dispenser_request_id="drop-monitor"
            )["state"],
            "monitoring",
        )

        resume = IncidentApprovalLatch()
        resume.open({"incident_id": "resume"})
        resume.decide(
            incident_id="resume",
            request_id="decision-resume",
            decision="drop_then_resume",
            operator_id="operator",
        )
        self.assertEqual(
            resume.mark_dispense_succeeded(
                dispenser_request_id="drop-resume"
            )["state"],
            "resuming",
        )

    def test_only_pre_actuation_failure_returns_to_approval(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        record = latch.mark_dispense_failed(
            dispenser_request_id="drop-1",
            result="communication_error",
            actuation_started=False,
        )
        self.assertEqual(record["state"], "approval_required")
        self.assertIsNone(record["decision"])

    def test_dispenser_request_id_is_required(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        with self.assertRaises(ValueError):
            latch.mark_dispense_succeeded(dispenser_request_id="  ")

    def test_progress_does_not_return_to_approval(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        for state in ("dispensing", "waiting", "homing"):
            record = latch.mark_dispense_progress(
                dispenser_request_id="decision-1",
                state=state,
            )
            self.assertEqual(record["state"], "dispensing")
            self.assertEqual(record["dispenser_progress"], state)

    def test_cancel_does_not_overwrite_resolved_incident(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision="resume",
            operator_id="operator",
        )
        latch.resolve_resume()
        self.assertIsNone(latch.cancel("mission complete"))
        self.assertEqual(latch.snapshot()["state"], "resolved")

    def test_field_check_requires_explicit_acknowledgement(self):
        latch = IncidentApprovalLatch()
        latch.open({"incident_id": "thermal-pump"})
        latch.decide(
            incident_id="thermal-pump",
            request_id="decision-1",
            decision=DECISION_DROP_THEN_MONITOR,
            operator_id="operator",
        )
        latch.mark_dispense_failed(
            dispenser_request_id="decision-1",
            result="jam_suspected",
            actuation_started=True,
        )
        self.assertTrue(latch.is_paused())
        record, created = latch.decide(
            incident_id="thermal-pump",
            request_id="field-check-1",
            decision=DECISION_ACKNOWLEDGE_FIELD_CHECK,
            operator_id="operator",
        )
        self.assertTrue(created)
        self.assertEqual(record["state"], "resuming")


if __name__ == "__main__":
    unittest.main()
