import unittest

from hazard_guard_dispenser.approval_auth import (
    command_authorization,
    valid_command_authorization,
)


class ApprovalAuthorizationTests(unittest.TestCase):
    def test_expected_contract_vector_and_validation(self):
        authorization = command_authorization(
            "test-secret",
            request_id="request-1",
            detection_id="thermal:abc",
        )
        self.assertEqual(
            authorization,
            "1b5ead978643e30c882ce99d37347e70854400fc73db8967fb2ccb114b018b1f",
        )
        self.assertTrue(
            valid_command_authorization(
                "test-secret",
                request_id="request-1",
                detection_id="thermal:abc",
                authorization=authorization,
            )
        )

    def test_changed_request_is_rejected(self):
        authorization = command_authorization(
            "test-secret",
            request_id="request-1",
            detection_id=None,
        )
        self.assertFalse(
            valid_command_authorization(
                "test-secret",
                request_id="request-2",
                detection_id=None,
                authorization=authorization,
            )
        )


if __name__ == "__main__":
    unittest.main()
