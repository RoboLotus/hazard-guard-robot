import unittest

from hazard_guard_mission_manager.dispenser_auth import (
    command_authorization,
    decision_authorization,
    valid_decision_authorization,
)


class DispenserAuthorizationContractTests(unittest.TestCase):
    def test_matches_dispenser_contract_vector(self):
        self.assertEqual(
            command_authorization(
                "test-secret",
                request_id="request-1",
                detection_id="thermal:abc",
            ),
            "1b5ead978643e30c882ce99d37347e70854400fc73db8967fb2ccb114b018b1f",
        )

    def test_decision_authorization_binds_all_approval_fields(self):
        authorization = decision_authorization(
            "test-secret",
            incident_id="incident-1",
            request_id="request-1",
            decision="drop_then_monitor",
            operator_id="operator-1",
        )
        self.assertTrue(
            valid_decision_authorization(
                "test-secret",
                incident_id="incident-1",
                request_id="request-1",
                decision="drop_then_monitor",
                operator_id="operator-1",
                authorization=authorization,
            )
        )
        self.assertFalse(
            valid_decision_authorization(
                "test-secret",
                incident_id="incident-1",
                request_id="request-1",
                decision="resume",
                operator_id="operator-1",
                authorization=authorization,
            )
        )


if __name__ == "__main__":
    unittest.main()
