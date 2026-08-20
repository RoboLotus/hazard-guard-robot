from pathlib import Path


def test_launch_exposes_safe_recording_controls():
    source = (Path(__file__).resolve().parent.parent / "launch" / "bag_record.launch.py").read_text(encoding="utf-8")
    for name in ("profile", "storage_root", "minimum_free_gb", "max_duration_seconds", "max_size_gb", "allow_experimental", "enable_control_services"):
        assert f'DeclareLaunchArgument("{name}"' in source
    assert "bag_session_manager" in source


def test_recorder_exposes_web_control_contract():
    source = (Path(__file__).resolve().parent.parent / "hazard_guard_bag_recorder" / "node.py").read_text(encoding="utf-8")
    assert '"/hazard_guard/bag/control"' in source
    assert "status_json" in source
    assert '"/hazard_guard/bag/status_json"' in source
    assert "effective_experimental" in source
    assert 'command == "list"' in source
    assert 'self.publish_status(self.status_payload()["state"])' in source
    assert "get_publishers_info_by_topic" in source


def test_manifest_declares_rosbag_and_launch_runtime_dependencies():
    manifest = (Path(__file__).resolve().parent.parent / "package.xml").read_text(encoding="utf-8")
    for dependency in (
        "launch",
        "launch_ros",
        "ros2bag",
        "rosbag2_storage_default_plugins",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in manifest
