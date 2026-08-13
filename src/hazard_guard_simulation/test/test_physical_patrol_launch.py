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


def test_physical_speed_profile_is_consistent_and_keeps_safe_acceleration():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    behaviors = params["behavior_server"]["ros__parameters"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert follow_path["max_vel_x"] == 0.14
    assert follow_path["max_vel_y"] == 0.12
    assert follow_path["max_vel_theta"] == 0.42
    assert follow_path["max_speed_xy"] == 0.14
    assert smoother["max_velocity"] == [0.14, 0.12, 0.42]
    assert smoother["min_velocity"] == [-0.10, -0.12, -0.42]
    assert behaviors["max_rotational_vel"] == 0.42

    # A higher velocity ceiling must not also make the physical robot jump to
    # that velocity more abruptly than the field-tested profile.
    assert [
        follow_path["acc_lim_x"],
        follow_path["acc_lim_y"],
        follow_path["acc_lim_theta"],
    ] == [0.4, 0.3, 0.8]
    assert [
        follow_path["decel_lim_x"],
        follow_path["decel_lim_y"],
        follow_path["decel_lim_theta"],
    ] == [-0.4, -0.3, -0.8]
    assert smoother["max_accel"] == [0.4, 0.3, 0.8]
    assert smoother["max_decel"] == [-0.4, -0.3, -0.8]


def test_initial_pose_burst_spans_nav2_activation_window():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'period=5.0' in source
    assert '"repeat_count": 3' in source
    assert '"interval_sec": 1.0' in source


def test_physical_mission_alignment_uses_relaxed_sampled_policy():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    mission = params["hazard_guard_mission_manager"]["ros__parameters"]
    source = LAUNCH.read_text(encoding="utf-8")

    assert mission["position_tolerance_m"] == 0.10
    assert mission["yaw_tolerance_rad"] == 0.10
    assert mission["acceptable_position_tolerance_m"] == 0.15
    assert mission["acceptable_yaw_tolerance_rad"] == 0.17
    assert mission["hard_position_tolerance_m"] == 0.25
    assert mission["hard_yaw_tolerance_rad"] == 0.261799
    assert mission["alignment_retries"] == 1
    assert mission["pose_sample_count"] == 5
    assert mission["pose_min_valid_samples"] == 3
    assert mission["pose_sample_interval_sec"] == 0.15
    assert "parameters=[nav2_params_file]" in source


def test_person_safety_is_opt_in_and_gates_only_motor_facing_cmd_vel():
    source = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n                "use_person_safety"' in source
    assert 'default_value="false"' in source
    assert '"physical_m1_bringup.launch.py"' in source
    assert '"motor_cmd_vel_topic": "/cmd_vel_safe"' in source
    assert 'condition=UnlessCondition(use_person_safety)' in source
    assert 'condition=IfCondition(use_person_safety)' in source
    assert '"hazard_guard_person_detection"' in source
    assert '"hazard_guard_safety_supervisor"' in source
    assert 'name="safety_supervision_enabled"' in source


def test_physical_person_detection_uses_hp60c_rgb_and_depth_topics():
    source = LAUNCH.read_text(encoding="utf-8")

    assert "/ascamera_hp60c/camera_publisher/rgb0/image" in source
    assert "/ascamera_hp60c/camera_publisher/depth0/image_raw" in source
    assert '"start_person_camera"' in source
    assert '"person_depth_registration_verified"' in source
    assert '"person_confidence"' in source
    assert '"person_image_size"' in source
    assert '"person_inference_rate_hz"' in source
    assert '"confidence": person_confidence' in source
    assert '"image_size": person_image_size' in source
    assert '"inference_rate_hz": person_inference_rate_hz' in source


def test_only_physical_motor_driver_consumes_gated_velocity() -> None:
    source = (PACKAGE / "launch" / "physical_m1_bringup.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'executable="Mcnamu_driver_M1"' in source
    assert 'remappings=[("cmd_vel", motor_cmd_vel_topic)]' in source
    assert 'Node(package="yahboomcar_ctrl", executable="yahboom_joy_M1")' in source
    assert "SetRemap" not in source
