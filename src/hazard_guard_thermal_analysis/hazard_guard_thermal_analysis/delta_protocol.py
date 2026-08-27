"""Versioned binary thermal-map deltas.

This module only defines the Robot-side wire contract.  The existing HGPC
snapshot transport remains the bootstrap and resynchronisation fallback.
All integer and floating-point fields use little-endian byte order.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct


MAGIC = b"HGTD"
PROTOCOL_VERSION = 1

_HEADER = struct.Struct("<4sHBBQQHH")
_COUNT = struct.Struct("<I")
_DYNAMIC_COUNTS = struct.Struct("<III")
_STATIC_UPDATE = struct.Struct("<Iff")
_DYNAMIC_UPDATE = struct.Struct("<iiiff")
_DYNAMIC_DELETE = struct.Struct("<iii")


class DeltaProtocolError(ValueError):
    """Raised when a thermal delta packet is malformed or unsupported."""


class PacketType(IntEnum):
    STATIC_THERMAL_DELTA = 1
    DYNAMIC_DELTA = 2


@dataclass(frozen=True)
class StaticThermalUpdate:
    voxel_index: int
    temperature_c: float
    confidence: float


@dataclass(frozen=True)
class DynamicThermalUpdate:
    key: tuple[int, int, int]
    temperature_c: float
    confidence: float


@dataclass(frozen=True)
class StaticThermalDelta:
    session_id: str
    geometry_fingerprint: str
    sequence: int
    base_sequence: int
    updates: tuple[StaticThermalUpdate, ...]
    protocol_version: int = PROTOCOL_VERSION
    packet_type: PacketType = PacketType.STATIC_THERMAL_DELTA


@dataclass(frozen=True)
class DynamicDelta:
    session_id: str
    geometry_fingerprint: str
    sequence: int
    base_sequence: int
    created: tuple[DynamicThermalUpdate, ...]
    updated: tuple[DynamicThermalUpdate, ...]
    deleted: tuple[tuple[int, int, int], ...]
    protocol_version: int = PROTOCOL_VERSION
    packet_type: PacketType = PacketType.DYNAMIC_DELTA


DeltaPacket = StaticThermalDelta | DynamicDelta


def _encode_header(packet: DeltaPacket) -> bytes:
    if packet.protocol_version != PROTOCOL_VERSION:
        raise DeltaProtocolError("unsupported protocol_version")
    if packet.sequence < 0 or packet.base_sequence < 0:
        raise DeltaProtocolError("sequence values must be non-negative")
    session = packet.session_id.encode("utf-8")
    fingerprint = packet.geometry_fingerprint.encode("utf-8")
    if len(session) > 0xFFFF or len(fingerprint) > 0xFFFF:
        raise DeltaProtocolError("session_id or geometry_fingerprint is too long")
    return (
        _HEADER.pack(
            MAGIC,
            packet.protocol_version,
            int(packet.packet_type),
            0,
            packet.sequence,
            packet.base_sequence,
            len(session),
            len(fingerprint),
        )
        + session
        + fingerprint
    )


def _validate_temperature_update(temperature_c: float, confidence: float) -> None:
    if not math.isfinite(temperature_c):
        raise DeltaProtocolError("temperature must be finite")
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise DeltaProtocolError("confidence must be finite and in [0, 1]")


def encode_delta(packet: DeltaPacket) -> bytes:
    payload = bytearray(_encode_header(packet))
    if isinstance(packet, StaticThermalDelta):
        payload.extend(_COUNT.pack(len(packet.updates)))
        for update in packet.updates:
            if not 0 <= update.voxel_index <= 0xFFFFFFFF:
                raise DeltaProtocolError("voxel_index does not fit uint32")
            _validate_temperature_update(update.temperature_c, update.confidence)
            payload.extend(_STATIC_UPDATE.pack(
                update.voxel_index, update.temperature_c, update.confidence
            ))
    elif isinstance(packet, DynamicDelta):
        payload.extend(_DYNAMIC_COUNTS.pack(
            len(packet.created), len(packet.updated), len(packet.deleted)
        ))
        for update in (*packet.created, *packet.updated):
            _validate_temperature_update(update.temperature_c, update.confidence)
            payload.extend(_DYNAMIC_UPDATE.pack(
                *update.key, update.temperature_c, update.confidence
            ))
        for key in packet.deleted:
            payload.extend(_DYNAMIC_DELETE.pack(*key))
    else:
        raise TypeError(f"unsupported delta packet: {type(packet)!r}")
    return bytes(payload)


def _take(view: memoryview, offset: int, record: struct.Struct) -> tuple[tuple, int]:
    end = offset + record.size
    if end > len(view):
        raise DeltaProtocolError("truncated delta packet")
    return record.unpack_from(view, offset), end


def decode_delta(data: bytes | bytearray | memoryview) -> DeltaPacket:
    view = memoryview(data)
    values, offset = _take(view, 0, _HEADER)
    magic, version, raw_type, flags, sequence, base_sequence, session_len, fp_len = values
    if magic != MAGIC:
        raise DeltaProtocolError("invalid delta packet magic")
    if version != PROTOCOL_VERSION:
        raise DeltaProtocolError("unsupported protocol_version")
    if flags != 0:
        raise DeltaProtocolError("unsupported delta packet flags")
    try:
        packet_type = PacketType(raw_type)
    except ValueError as exc:
        raise DeltaProtocolError("unsupported packet type") from exc
    strings_end = offset + session_len + fp_len
    if strings_end > len(view):
        raise DeltaProtocolError("truncated delta packet metadata")
    try:
        session_id = bytes(view[offset:offset + session_len]).decode("utf-8")
        offset += session_len
        fingerprint = bytes(view[offset:offset + fp_len]).decode("utf-8")
        offset += fp_len
    except UnicodeDecodeError as exc:
        raise DeltaProtocolError("invalid UTF-8 packet metadata") from exc

    common = dict(
        session_id=session_id,
        geometry_fingerprint=fingerprint,
        sequence=sequence,
        base_sequence=base_sequence,
        protocol_version=version,
        packet_type=packet_type,
    )
    if packet_type is PacketType.STATIC_THERMAL_DELTA:
        (count,), offset = _take(view, offset, _COUNT)
        updates = []
        for _ in range(count):
            values, offset = _take(view, offset, _STATIC_UPDATE)
            updates.append(StaticThermalUpdate(*values))
        packet: DeltaPacket = StaticThermalDelta(
            updates=tuple(updates), **common
        )
    else:
        counts, offset = _take(view, offset, _DYNAMIC_COUNTS)
        created_count, updated_count, deleted_count = counts
        changed = []
        for _ in range(created_count + updated_count):
            values, offset = _take(view, offset, _DYNAMIC_UPDATE)
            changed.append(DynamicThermalUpdate(values[:3], values[3], values[4]))
        deleted = []
        for _ in range(deleted_count):
            values, offset = _take(view, offset, _DYNAMIC_DELETE)
            deleted.append(values)
        packet = DynamicDelta(
            created=tuple(changed[:created_count]),
            updated=tuple(changed[created_count:]),
            deleted=tuple(deleted),
            **common,
        )
    if offset != len(view):
        raise DeltaProtocolError("unexpected trailing delta packet data")
    return packet
