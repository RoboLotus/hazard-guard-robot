from pathlib import Path


def test_launch_exposes_safe_recording_controls():
    source = (Path(__file__).resolve().parent.parent / "launch" / "bag_record.launch.py").read_text(encoding="utf-8")
    for name in ("profile", "storage_root", "minimum_free_gb", "max_duration_seconds", "max_size_gb", "allow_experimental"):
        assert f'DeclareLaunchArgument("{name}"' in source
    assert "bag_session_manager" in source
