from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_package_installs_frozen_map_ros_entry_point() -> None:
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert '"frozen_thermal_map = "' in setup
    assert "hazard_guard_thermal_analysis.frozen_map_node:main" in setup


def test_frozen_map_node_contract_is_cumulative_and_does_not_publish_tf() -> None:
    source = (
        PACKAGE
        / "hazard_guard_thermal_analysis"
        / "frozen_map_node.py"
    ).read_text(encoding="utf-8")

    assert '"/hazard_guard/thermal/points"' in source
    assert '"/hazard_guard/thermal/map"' in source
    assert '"/hazard_guard/thermal/dynamic"' in source
    assert '"/hazard_guard/thermal/map/status"' in source
    assert '"cumulative": True' in source
    assert '"analysis_input_route"' in source
    assert "self._static_observation_publisher.publish(message)" in source
    assert "if self._layer is None:" in source
    assert "if not self._localization_gate.ready:" in source
    assert 'message.header.frame_id != expected_frame' in source
    assert "Time.from_msg(message.header.stamp)" in source
    assert "TransformBroadcaster" not in source
    assert "StaticTransformBroadcaster" not in source


def test_status_exposes_map_identity_persistence_and_match_metrics() -> None:
    source = (
        PACKAGE
        / "hazard_guard_thermal_analysis"
        / "frozen_map_node.py"
    ).read_text(encoding="utf-8")
    for key in (
        "fixed_map_available",
        "session_id",
        "observed_voxel_count",
        "match_ratio",
        "rejected_observation_count",
        "persisted_at",
        "state_path",
        "fingerprint",
        "dynamic_active_voxel_count",
        "dynamic_confirmed_voxel_count",
        "dynamic_hit_voxel_count",
        "dynamic_visible_miss_voxel_count",
        "dynamic_removed_voxel_count",
        "dynamic_state_path",
    ):
        assert f'"{key}"' in source


def test_last_observation_timestamp_changes_only_after_accepted_update() -> None:
    source = (
        PACKAGE
        / "hazard_guard_thermal_analysis"
        / "frozen_map_node.py"
    ).read_text(encoding="utf-8")
    result_branch = source.split(
        "        if result.accepted or dynamic_updated:\n", 1
    )[1]
    accepted, rejected = result_branch.split("        else:\n", 1)
    rejected = rejected.split("        self._publish_status()\n", 1)[0]
    assert "self._last_observation_at_ns" in accepted
    assert "self._last_observation_at_ns" not in rejected


def test_stale_receipt_is_consumed_instead_of_retried() -> None:
    source = (
        PACKAGE
        / "hazard_guard_thermal_analysis"
        / "fusion_node.py"
    ).read_text(encoding="utf-8")

    stale_branch = source.index("if pair_delta > tolerance:")
    consume = source.index(
        "self._last_used_depth_receipt = depth_receipt", stale_branch
    )
    warning = source.index("self.get_logger().warning(", stale_branch)
    assert stale_branch < consume < warning
