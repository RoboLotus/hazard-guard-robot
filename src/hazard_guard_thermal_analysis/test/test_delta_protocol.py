import struct

import pytest

from hazard_guard_thermal_analysis.delta_protocol import (
    DeltaProtocolError,
    DynamicDelta,
    DynamicThermalUpdate,
    PacketType,
    StaticThermalDelta,
    StaticThermalUpdate,
    decode_delta,
    encode_delta,
)


def test_static_delta_round_trip() -> None:
    original = StaticThermalDelta(
        session_id="patrol-2026-08-27",
        geometry_fingerprint="a" * 64,
        sequence=42,
        base_sequence=41,
        updates=(
            StaticThermalUpdate(3, 42.5, 0.75),
            StaticThermalUpdate(100_000, -5.25, 1.0),
        ),
    )

    encoded = encode_delta(original)
    decoded = decode_delta(encoded)

    assert encoded[:4] == b"HGTD"
    assert decoded == original
    assert decoded.packet_type is PacketType.STATIC_THERMAL_DELTA


def test_dynamic_delta_round_trip() -> None:
    original = DynamicDelta(
        session_id="patrol-1",
        geometry_fingerprint="fixed-map-fingerprint",
        sequence=8,
        base_sequence=7,
        created=(DynamicThermalUpdate((20, -2, 4), 80.0, 0.5),),
        updated=(DynamicThermalUpdate((-1, 0, 2), 36.25, 0.875),),
        deleted=((10, 11, -12), (100, 200, 300)),
    )

    decoded = decode_delta(encode_delta(original))

    assert decoded == original
    assert decoded.packet_type is PacketType.DYNAMIC_DELTA


@pytest.mark.parametrize(
    "payload, message",
    [
        (b"bad", "truncated"),
        (b"NOPE" + bytes(28), "magic"),
    ],
)
def test_decode_rejects_malformed_packets(payload: bytes, message: str) -> None:
    with pytest.raises(DeltaProtocolError, match=message):
        decode_delta(payload)


def test_decode_rejects_unsupported_version_and_trailing_bytes() -> None:
    packet = encode_delta(StaticThermalDelta(
        session_id="s",
        geometry_fingerprint="f",
        sequence=1,
        base_sequence=0,
        updates=(),
    ))
    unsupported = bytearray(packet)
    struct.pack_into("<H", unsupported, 4, 2)

    with pytest.raises(DeltaProtocolError, match="protocol_version"):
        decode_delta(unsupported)
    with pytest.raises(DeltaProtocolError, match="trailing"):
        decode_delta(packet + b"x")


def test_encode_validates_static_values() -> None:
    packet = StaticThermalDelta(
        session_id="s",
        geometry_fingerprint="f",
        sequence=1,
        base_sequence=0,
        updates=(StaticThermalUpdate(1, 20.0, 1.5),),
    )
    with pytest.raises(DeltaProtocolError, match="confidence"):
        encode_delta(packet)
