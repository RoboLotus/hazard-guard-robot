from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_gazebo_bridge_can_consume_the_gated_velocity_topic() -> None:
    source = (PACKAGE / "launch" / "simulation.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'LaunchConfiguration("cmd_vel_ros_topic")' in source
    assert '("/cmd_vel", cmd_vel_ros_topic)' in source


def test_sim_navigation_uses_registered_rgb_depth_and_opt_in_safety() -> None:
    source = (PACKAGE / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'LaunchConfiguration("use_person_safety")' in source
    assert '"rgb_topic": "/camera/image_raw"' in source
    assert '"depth_topic": "/depth_camera/image_raw"' in source
    assert "'/cmd_vel_safe' if '" in source
    assert '"safety_supervision_enabled"' in source


def test_sim_localization_has_the_same_person_safety_contract() -> None:
    source = (PACKAGE / "launch" / "localization.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'LaunchConfiguration("use_person_safety")' in source
    assert '"rgb_topic": "/camera/image_raw"' in source
    assert '"depth_topic": "/depth_camera/image_raw"' in source
    assert '"cmd_vel_ros_topic": bridge_velocity_topic' in source
