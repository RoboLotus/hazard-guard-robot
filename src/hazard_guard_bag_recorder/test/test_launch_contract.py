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
