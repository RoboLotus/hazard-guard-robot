import numpy as np

from hazard_guard_thermal_analysis.delta_protocol import (
    DynamicDelta,
    StaticThermalDelta,
    decode_delta,
)
from hazard_guard_thermal_analysis.delta_publisher import ThermalDeltaPublisher
from hazard_guard_thermal_analysis.dynamic_map import DynamicUpdateResult
from hazard_guard_thermal_analysis.frozen_map import (
    FixedGeometry,
    FrozenThermalLayer,
    VoxelHashIndex,
)


def _static_layer(x: float = 0.0) -> FrozenThermalLayer:
    geometry = FixedGeometry.from_points(
        np.asarray([[x, 0.0, 0.0]], dtype=np.float32),
        voxel_size_m=0.01,
        maximum_voxels=10,
    )
    return FrozenThermalLayer(
        geometry, VoxelHashIndex(geometry, cell_size_m=0.08)
    )


def _update_static(layer: FrozenThermalLayer, temperature: float = 25.0) -> None:
    layer.integrate(
        layer.geometry.points.copy(),
        np.asarray([temperature], dtype=np.float32),
        np.asarray([0.8], dtype=np.float32),
        observed_at_ns=10,
        minimum_match_ratio=1.0,
        minimum_observations=1,
    )


def _dynamic_result(**changes) -> DynamicUpdateResult:
    counts = dict(
        valid_observation_count=1,
        static_observation_count=0,
        candidate_voxel_count=1,
        clustered_candidate_voxel_count=1,
        hit_voxel_count=1,
        visible_miss_voxel_count=0,
        removed_voxel_count=0,
        active_voxel_count=1,
        confirmed_voxel_count=1,
    )
    return DynamicUpdateResult(**counts, **changes)


class _DynamicValues:
    def __init__(self, values):
        self.values = values

    def thermal_values(self, keys):
        return {key: self.values[key] for key in keys if key in self.values}


def test_static_dirty_emits_real_packet_and_no_change_emits_nothing() -> None:
    published = []
    publisher = ThermalDeltaPublisher(published.append)
    layer = _static_layer()
    _update_static(layer)

    emitted = publisher.publish_pending(
        session_id="session-a",
        static_layer=layer,
        dynamic_layer=None,
        dynamic_result=None,
    )

    assert emitted == tuple(published)
    packet = decode_delta(published[0])
    assert isinstance(packet, StaticThermalDelta)
    assert packet.sequence == 1
    assert packet.base_sequence == 0
    assert packet.updates[0].voxel_index == 0
    assert packet.updates[0].temperature_c == 25.0
    assert publisher.publish_pending(
        session_id="session-a",
        static_layer=layer,
        dynamic_layer=None,
        dynamic_result=None,
    ) == ()
    assert len(published) == 1


def test_dynamic_create_update_delete_emit_ordered_packets() -> None:
    published = []
    publisher = ThermalDeltaPublisher(published.append)
    layer = _static_layer()
    dynamic = _DynamicValues({(1, 2, 3): (42.0, 0.75)})

    for result in (
        _dynamic_result(created_keys=((1, 2, 3),)),
        _dynamic_result(updated_keys=((1, 2, 3),)),
        _dynamic_result(deleted_keys=((1, 2, 3),)),
    ):
        publisher.publish_pending(
            session_id="session-a",
            static_layer=layer,
            dynamic_layer=dynamic,
            dynamic_result=result,
        )

    packets = [decode_delta(packet) for packet in published]
    assert all(isinstance(packet, DynamicDelta) for packet in packets)
    assert [(packet.base_sequence, packet.sequence) for packet in packets] == [
        (0, 1), (1, 2), (2, 3)
    ]
    assert packets[0].created[0].key == (1, 2, 3)
    assert packets[1].updated[0].temperature_c == 42.0
    assert packets[2].deleted == ((1, 2, 3),)


def test_session_and_fingerprint_changes_reset_sequence() -> None:
    published = []
    publisher = ThermalDeltaPublisher(published.append)
    first = _static_layer(0.0)
    _update_static(first)
    publisher.publish_pending(
        session_id="session-a", static_layer=first,
        dynamic_layer=None, dynamic_result=None,
    )

    second = _static_layer(1.0)
    _update_static(second)
    publisher.publish_pending(
        session_id="session-a", static_layer=second,
        dynamic_layer=None, dynamic_result=None,
    )
    _update_static(second, 30.0)
    publisher.publish_pending(
        session_id="session-b", static_layer=second,
        dynamic_layer=None, dynamic_result=None,
    )

    packets = [decode_delta(packet) for packet in published]
    assert [packet.sequence for packet in packets] == [1, 1, 1]
    assert packets[0].geometry_fingerprint != packets[1].geometry_fingerprint
    assert packets[2].session_id == "session-b"


def test_failed_static_publish_keeps_dirty_indices_and_sequence() -> None:
    layer = _static_layer()
    _update_static(layer)

    def fail(_packet):
        raise RuntimeError("ROS publisher unavailable")

    publisher = ThermalDeltaPublisher(fail)
    try:
        publisher.publish_pending(
            session_id="session-a", static_layer=layer,
            dynamic_layer=None, dynamic_result=None,
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("publication should fail")

    assert publisher.sequence == 0
    assert layer.pending_dirty_indices == (0,)
