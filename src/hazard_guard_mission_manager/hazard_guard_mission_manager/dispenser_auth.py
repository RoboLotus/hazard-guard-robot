from __future__ import annotations

import hashlib
import hmac


def command_authorization(
    secret: str,
    *,
    request_id: str,
    detection_id: str | None,
) -> str:
    payload = f"drop\n{request_id}\n{detection_id or ''}".encode("utf-8")
    return hmac.new(secret.encode("utf-8"), payload, hashlib.sha256).hexdigest()


def decision_authorization(
    secret: str,
    *,
    incident_id: str,
    request_id: str,
    decision: str,
    operator_id: str,
) -> str:
    payload = (
        f"decision\n{incident_id}\n{request_id}\n{decision}\n{operator_id}"
    ).encode("utf-8")
    return hmac.new(secret.encode("utf-8"), payload, hashlib.sha256).hexdigest()


def valid_decision_authorization(
    secret: str,
    *,
    incident_id: str,
    request_id: str,
    decision: str,
    operator_id: str,
    authorization: str,
) -> bool:
    if not secret or not authorization:
        return False
    expected = decision_authorization(
        secret,
        incident_id=incident_id,
        request_id=request_id,
        decision=decision,
        operator_id=operator_id,
    )
    return hmac.compare_digest(expected, authorization)
