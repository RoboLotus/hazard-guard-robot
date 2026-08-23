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


def valid_command_authorization(
    secret: str,
    *,
    request_id: str,
    detection_id: str | None,
    authorization: str,
) -> bool:
    if not secret or not authorization:
        return False
    expected = command_authorization(
        secret,
        request_id=request_id,
        detection_id=detection_id,
    )
    return hmac.compare_digest(expected, authorization)
