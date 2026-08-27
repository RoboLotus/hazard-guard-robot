"""Robot-side construction and publication of thermal map deltas."""

from __future__ import annotations

from collections.abc import Callable
import math

from .delta_protocol import (
    DynamicDelta,
    DynamicThermalUpdate,
    StaticThermalDelta,
    StaticThermalUpdate,
    encode_delta,
)
from .dynamic_map import DynamicUpdateResult, DynamicVoxelLayer
from .frozen_map import FrozenThermalLayer


class ThermalDeltaPublisher:
    """Build ordered packets and pass their bytes to one ROS publisher.

    Sequence is shared by static and dynamic packets on the single delta
    topic.  For every emitted packet ``base_sequence`` is the preceding
    successfully published sequence and ``sequence == base_sequence + 1``.
    Changing either session id or geometry fingerprint starts a new state
    identity and resets the pair to ``base=0, sequence=1``.
    """

    def __init__(self, publish_bytes: Callable[[bytes], None]) -> None:
        self._publish_bytes = publish_bytes
        self._session_id = ""
        self._geometry_fingerprint = ""
        self._sequence = 0

    @property
    def sequence(self) -> int:
        return self._sequence

    @property
    def identity(self) -> tuple[str, str]:
        return self._session_id, self._geometry_fingerprint

    def reset(self, session_id: str, geometry_fingerprint: str) -> None:
        self._session_id = str(session_id)
        self._geometry_fingerprint = str(geometry_fingerprint)
        self._sequence = 0

    def _ensure_identity(self, session_id: str, fingerprint: str) -> None:
        identity = (str(session_id), str(fingerprint))
        if identity != self.identity:
            self.reset(*identity)

    def _emit(self, factory: Callable[[int, int], object]) -> bytes:
        base = self._sequence
        sequence = base + 1
        packet = encode_delta(factory(sequence, base))
        self._publish_bytes(packet)
        # Failed ROS publication raises before advancing, so the next retry
        # keeps a continuous sequence rather than silently creating a gap.
        self._sequence = sequence
        return packet

    def publish_pending(
        self,
        *,
        session_id: str,
        static_layer: FrozenThermalLayer,
        dynamic_layer: DynamicVoxelLayer | None,
        dynamic_result: DynamicUpdateResult | None,
    ) -> tuple[bytes, ...]:
        fingerprint = static_layer.geometry.fingerprint
        self._ensure_identity(session_id, fingerprint)
        emitted: list[bytes] = []

        static_indices = static_layer.pending_dirty_indices
        if static_indices:
            updates = tuple(
                StaticThermalUpdate(
                    index,
                    float(static_layer.temperature_c[index]),
                    float(static_layer.confidence[index]),
                )
                for index in static_indices
            )
            emitted.append(self._emit(lambda sequence, base: StaticThermalDelta(
                session_id=self._session_id,
                geometry_fingerprint=self._geometry_fingerprint,
                sequence=sequence,
                base_sequence=base,
                updates=updates,
            )))
            # Drain only after successful publication.  A publisher exception
            # leaves the change set pending for a later retry.
            static_layer.drain_dirty_indices()

        if dynamic_layer is not None and dynamic_result is not None:
            changed_keys = dynamic_result.created_keys + dynamic_result.updated_keys
            values = dynamic_layer.thermal_values(changed_keys)

            def dynamic_updates(keys):
                return tuple(
                    DynamicThermalUpdate(key, temperature, confidence)
                    for key in keys
                    if key in values
                    for temperature, confidence in (values[key],)
                    if math.isfinite(temperature)
                    and math.isfinite(confidence)
                    and 0.0 <= confidence <= 1.0
                )

            created = dynamic_updates(dynamic_result.created_keys)
            updated = dynamic_updates(dynamic_result.updated_keys)
            deleted = dynamic_result.deleted_keys
            if created or updated or deleted:
                emitted.append(self._emit(lambda sequence, base: DynamicDelta(
                    session_id=self._session_id,
                    geometry_fingerprint=self._geometry_fingerprint,
                    sequence=sequence,
                    base_sequence=base,
                    created=created,
                    updated=updated,
                    deleted=deleted,
                )))
        return tuple(emitted)
