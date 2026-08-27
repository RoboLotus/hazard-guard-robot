from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2

from hazard_guard_thermal_analysis.frozen_map import (
    thermal_analysis_input_route,
)
from hazard_guard_thermal_analysis.frozen_map_node import FrozenThermalMapNode


class _PublisherProbe:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _write_single_point_ply(path: Path) -> None:
    path.write_text(
        "\n".join(
            (
                "ply",
                "format ascii 1.0",
                "element vertex 1",
                "property float x",
                "property float y",
                "property float z",
                "end_header",
                "0 0 0",
            )
        ),
        encoding="ascii",
    )


def test_resolved_self_relay_topic_fails_before_cloud_can_multiply() -> None:
    rclpy.init()
    try:
        with pytest.raises(ValueError, match="resolve to the same topic"):
            FrozenThermalMapNode(
                namespace="/thermal",
                parameter_overrides=[
                    Parameter("input_topic", value="points"),
                    Parameter(
                        "static_observation_output_topic",
                        value="/thermal/points",
                    ),
                ],
            )
    finally:
        rclpy.shutdown()


def test_missing_ply_relays_raw_then_switches_when_geometry_appears(
    tmp_path: Path,
) -> None:
    rclpy.init()
    map_path = tmp_path / "delayed-map.ply"
    node = FrozenThermalMapNode(
        parameter_overrides=[
            Parameter("map_cloud_path", value=str(map_path)),
            Parameter("localization_stable_samples", value=1),
        ]
    )
    probe = _PublisherProbe()
    node._static_observation_publisher = probe
    try:
        node._localization_gate.observe(
            np.zeros(3, dtype=np.float64),
            np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )
        message = PointCloud2()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        node._on_observation(message)
        node._process_pending_observation()

        assert probe.messages == [message]
        assert thermal_analysis_input_route(
            fixed_map_available=node._layer is not None
        ) == "raw_fallback"

        _write_single_point_ply(map_path)
        node._retry_geometry_load()
        assert thermal_analysis_input_route(
            fixed_map_available=node._layer is not None
        ) == "static_surface"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_ply_load_failure_still_relays_raw_observation(
    tmp_path: Path,
) -> None:
    map_path = tmp_path / "invalid-map.ply"
    map_path.write_text("not a ply", encoding="ascii")
    rclpy.init()
    node = FrozenThermalMapNode(
        parameter_overrides=[
            Parameter("map_cloud_path", value=str(map_path)),
            Parameter("localization_stable_samples", value=1),
        ]
    )
    probe = _PublisherProbe()
    node._static_observation_publisher = probe
    try:
        assert node._layer is None
        assert node._map_error
        failed_signature = node._last_failed_geometry_signature
        node._retry_geometry_load()
        assert node._layer is None
        assert node._last_failed_geometry_signature == failed_signature
        node._localization_gate.observe(
            np.zeros(3, dtype=np.float64),
            np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )
        message = PointCloud2()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        node._on_observation(message)
        node._process_pending_observation()

        assert probe.messages == [message]
        assert thermal_analysis_input_route(
            fixed_map_available=node._layer is not None
        ) == "raw_fallback"

        _write_single_point_ply(map_path)
        node._retry_geometry_load()
        assert thermal_analysis_input_route(
            fixed_map_available=node._layer is not None
        ) == "static_surface"
    finally:
        node.destroy_node()
        rclpy.shutdown()
