from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
LAUNCH = PACKAGE / "launch" / "physical_patrol.launch.py"
PARAMS = PACKAGE / "config" / "physical_nav2.yaml"


def test_physical_patrol_uses_stock_nav2_with_explicit_parameters():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'get_package_share_directory("nav2_bringup")' in source
    assert '"bringup_launch.py"' in source
    assert 'LaunchConfiguration("nav2_params_file")' in source
    assert '"params_file": nav2_params_file' in source
    assert 'DeclareLaunchArgument(\n                "params_file"' not in source
    assert '"map": map_path' in source
    assert '"slam": "False"' in source
    assert '"use_composition": "False"' in source
    assert "navigation_dwa_launch.py" not in source


def test_physical_nav2_parameters_cover_localization_and_dwb():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    amcl = params["amcl"]["ros__parameters"]
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert amcl["base_frame_id"] == "base_footprint"
    assert controller["controller_plugins"] == ["FollowPath"]
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert follow_path["critics"]
    assert follow_path["max_vel_y"] > 0.0
    assert params["map_server"]["ros__parameters"]["yaml_filename"] == ""


def test_initial_pose_burst_spans_nav2_activation_window():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'period=5.0' in source
    assert '"repeat_count": 3' in source
    assert '"interval_sec": 1.0' in source
