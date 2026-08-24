"""WebUI-managed physical M1 localization and Nav2 patrol stack."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(
    package: str,
    filename: str,
    arguments: Mapping[str, Any] | None = None,
    condition: Any | None = None,
) -> IncludeLaunchDescription:
    package_share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    map_path = LaunchConfiguration("map")
    # Do not call this launch argument "params_file". The included YDLIDAR
    # launch uses that generic name too, and launch configurations are visible
    # to nested includes. Sharing the name makes the lidar read Nav2's YAML.
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    use_person_safety = LaunchConfiguration("use_person_safety")
    start_person_camera = LaunchConfiguration("start_person_camera")
    person_model_path = LaunchConfiguration("person_model_path")
    person_device = LaunchConfiguration("person_device")
    person_confidence = LaunchConfiguration("person_confidence")
    person_image_size = LaunchConfiguration("person_image_size")
    person_inference_rate_hz = LaunchConfiguration("person_inference_rate_hz")
    person_depth_registration_verified = LaunchConfiguration(
        "person_depth_registration_verified"
    )
    use_performance_monitor = LaunchConfiguration("use_performance_monitor")
    use_dispenser = LaunchConfiguration("use_dispenser")
    enable_physical_drop = LaunchConfiguration("enable_physical_drop")
    enable_hazard_approval = LaunchConfiguration("enable_hazard_approval")
    dispenser_rear_offset_m = LaunchConfiguration("dispenser_rear_offset_m")
    performance_storage_path = LaunchConfiguration("performance_storage_path")
    enable_rgbd_mapping = LaunchConfiguration("enable_rgbd_mapping")
    enable_thermal_pipeline = LaunchConfiguration("enable_thermal_pipeline")
    enable_frozen_thermal_map = LaunchConfiguration(
        "enable_frozen_thermal_map"
    )
    start_thermal_pipeline = IfCondition(
        PythonExpression(
            [
                "'true' if '",
                enable_thermal_pipeline,
                "'.lower() == 'true' or '",
                enable_frozen_thermal_map,
                "'.lower() == 'true' else 'false'",
            ]
        )
    )
    start_hp60c_camera = IfCondition(
        PythonExpression(
            [
                "'true' if '",
                enable_rgbd_mapping,
                "'.lower() == 'true' or '",
                enable_frozen_thermal_map,
                "'.lower() == 'true' or ('",
                use_person_safety,
                "'.lower() == 'true' and '",
                start_person_camera,
                "'.lower() == 'true') else 'false'",
            ]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(
                    simulation_share / "config" / "physical_nav2.yaml"
                ),
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "use_person_safety",
                default_value="false",
                description=(
                    "Enable YOLO person detection, Nav2 speed limiting, and "
                    "the final fail-safe cmd_vel gate after the RGB-D safety "
                    "path has been validated."
                ),
            ),
            DeclareLaunchArgument(
                "start_person_camera",
                default_value="true",
                description=(
                    "Start the HP60C driver with person safety. Set false if "
                    "the camera driver is already running."
                ),
            ),
            DeclareLaunchArgument("person_model_path", default_value="yolo11n.pt"),
            DeclareLaunchArgument("person_device", default_value="0"),
            DeclareLaunchArgument("person_confidence", default_value="0.4"),
            DeclareLaunchArgument("person_image_size", default_value="640"),
            DeclareLaunchArgument(
                "person_inference_rate_hz",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "person_depth_registration_verified",
                default_value="false",
                description=(
                    "Set true only after verifying HP60C RGB-depth pixel "
                    "registration. Safety remains fail-closed while false."
                ),
            ),
            DeclareLaunchArgument("enable_rgbd_mapping", default_value="false"),
            DeclareLaunchArgument("use_performance_monitor", default_value="true"),
            DeclareLaunchArgument("performance_storage_path", default_value=""),
            DeclareLaunchArgument(
                "use_dispenser",
                default_value="false",
                description="Start the rear beacon dispenser node.",
            ),
            DeclareLaunchArgument(
                "enable_physical_drop",
                default_value="false",
                description=(
                    "Enable real servo motion only after the physical safety "
                    "checklist and approval secret are configured."
                ),
            ),
            DeclareLaunchArgument(
                "enable_hazard_approval",
                default_value="false",
                description=(
                    "Pause patrol on correlated thermal warning/critical "
                    "results and require an authenticated operator decision."
                ),
            ),
            DeclareLaunchArgument(
                "dispenser_rear_offset_m",
                default_value="-0.25",
                description=(
                    "Signed base_link X offset from robot center to the "
                    "dispenser outlet used for installed-beacon map markers."
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_database_path",
                default_value="/tmp/hazard_guard_physical_rgbd.db",
            ),
            DeclareLaunchArgument(
                "rtabmap_reset_database",
                default_value="false",
                description=(
                    "Reset only rtabmap_database_path when starting the "
                    "optional RGB-D capture"
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_storage_path",
                default_value="/tmp",
            ),
            DeclareLaunchArgument(
                "rgbd_cloud_stamp_mode",
                default_value="offset",
                choices=["preserve", "offset"],
            ),
            DeclareLaunchArgument(
                "rgbd_cloud_stamp_offset_sec",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "enable_thermal_pipeline", default_value="false"
            ),
            DeclareLaunchArgument(
                "enable_frozen_thermal_map",
                default_value="false",
                description=(
                    "Keep an existing PLY immutable, accumulate its thermal "
                    "attributes, and track persistent dynamic thermal voxels. "
                    "This also starts the live thermal-depth pipeline."
                ),
            ),
            DeclareLaunchArgument(
                "thermal_map_cloud_path",
                default_value="",
                description="Fixed RTAB-Map PLY used as immutable geometry",
            ),
            DeclareLaunchArgument(
                "thermal_map_state_path",
                default_value="",
                description=(
                    "Atomic NPZ checkpoint for the cumulative thermal layer"
                ),
            ),
            DeclareLaunchArgument(
                "thermal_dynamic_state_path",
                default_value="",
                description=(
                    "Atomic NPZ checkpoint for dynamic voxels. When empty, "
                    "derive a .dynamic.npz sibling from thermal_map_state_path"
                ),
            ),
            DeclareLaunchArgument(
                "thermal_map_session_id",
                default_value="",
                description="Owning map session identifier for status routing",
            ),
            DeclareLaunchArgument("thermal_roi_config", default_value=""),
            DeclareLaunchArgument(
                "thermal_baseline_path",
                default_value=(
                    "~/.local/share/hazard_guard/"
                    "physical_thermal_baselines.json"
                ),
            ),
            DeclareLaunchArgument(
                "thermal_baseline_collection_path",
                default_value=(
                    "~/.local/share/hazard_guard/"
                    "physical_thermal_baseline_collection.json"
                ),
            ),
            DeclareLaunchArgument(
                "thermal_baseline_minimum_valid_visits",
                default_value="8",
            ),
            DeclareLaunchArgument(
                "thermal_history_path",
                default_value="~/.local/share/hazard_guard/physical_thermal_history.jsonl",
            ),
            DeclareLaunchArgument(
                "thermal_air_temperature_topic", default_value=""
            ),
            DeclareLaunchArgument(
                "thermal_oil_temperature_topic", default_value=""
            ),
            DeclareLaunchArgument(
                "thermal_sensor_timeout_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "thermal_image_topic",
                default_value="/thermal_camera/image_raw",
            ),
            DeclareLaunchArgument(
                "thermal_info_topic",
                default_value="/thermal_camera/camera_info",
            ),
            DeclareLaunchArgument(
                "thermal_depth_image_topic",
                default_value="/depth_camera/image_raw",
            ),
            DeclareLaunchArgument(
                "thermal_depth_info_topic",
                default_value="/depth_camera/camera_info",
            ),
            DeclareLaunchArgument("thermal_scale", default_value="1.0"),
            DeclareLaunchArgument("thermal_offset_c", default_value="0.0"),
            SetLaunchConfiguration("use_sim_time", "false"),
            # Preserve the field-tested vendor bringup exactly when the new
            # safety feature is disabled.
            include(
                "yahboomcar_nav",
                "laser_bringup_launch.py",
                condition=UnlessCondition(use_person_safety),
            ),
            include(
                "hazard_guard_simulation",
                "physical_m1_bringup.launch.py",
                {"motor_cmd_vel_topic": "/cmd_vel_safe"},
                condition=IfCondition(use_person_safety),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ascamera"), "launch", "hp60c.launch.py"]
                    )
                ),
                condition=start_hp60c_camera,
            ),
            include(
                "hazard_guard_simulation",
                "rgbd_capture_after_localization.launch.py",
                {
                    "backend": "real",
                    "use_sim_time": "false",
                    "map": map_path,
                    "database_path": LaunchConfiguration(
                        "rtabmap_database_path"
                    ),
                    "reset_database": LaunchConfiguration(
                        "rtabmap_reset_database"
                    ),
                    "storage_path": LaunchConfiguration(
                        "rtabmap_storage_path"
                    ),
                    "cloud_stamp_mode": LaunchConfiguration(
                        "rgbd_cloud_stamp_mode"
                    ),
                    "cloud_stamp_offset_sec": LaunchConfiguration(
                        "rgbd_cloud_stamp_offset_sec"
                    ),
                    "readiness_timeout_sec": "60.0",
                },
                condition=IfCondition(enable_rgbd_mapping),
            ),
            include(
                "hazard_guard_person_detection",
                "person_detection.launch.py",
                {
                    "rgb_topic": "/ascamera_hp60c/camera_publisher/rgb0/image",
                    "depth_topic": (
                        "/ascamera_hp60c/camera_publisher/depth0/image_raw"
                    ),
                    "model_path": person_model_path,
                    "device": person_device,
                    "confidence": person_confidence,
                    "image_size": person_image_size,
                    "inference_rate_hz": person_inference_rate_hz,
                    "simulated": "false",
                    "depth_registration_verified": (
                        person_depth_registration_verified
                    ),
                    "use_sim_time": "false",
                },
                condition=IfCondition(use_person_safety),
            ),
            include(
                "hazard_guard_safety_supervisor",
                "person_safety.launch.py",
                {"use_sim_time": "false"},
                condition=IfCondition(use_person_safety),
            ),
            include(
                "hazard_guard_simulation",
                "physical_thermal_camera.launch.py",
                {"show_gui": "false"},
                condition=start_thermal_pipeline,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_path,
                    "params_file": nav2_params_file,
                    # Humble bringup evaluates this through PythonExpression.
                    "slam": "False",
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "false",
                }.items(),
            ),
            GroupAction(
                actions=[
                    SetParameter(
                        name="safety_supervision_enabled",
                        value=ParameterValue(
                            use_person_safety,
                            value_type=bool,
                        ),
                    ),
                    Node(
                        package="hazard_guard_mission_manager",
                        executable="mission_manager",
                        name="hazard_guard_mission_manager",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "hazard_approval_enabled": ParameterValue(
                                    enable_hazard_approval,
                                    value_type=bool,
                                ),
                                "dispenser_rear_offset_m": ParameterValue(
                                    dispenser_rear_offset_m,
                                    value_type=float,
                                ),
                            },
                        ],
                    ),
                    Node(
                        package="hazard_guard_performance_monitor",
                        executable="performance_monitor",
                        name="hazard_guard_performance_monitor",
                        output="screen",
                        condition=IfCondition(use_performance_monitor),
                        parameters=[
                            {
                                "storage_path": performance_storage_path,
                                "sample_interval_sec": 1.0,
                            }
                        ],
                    ),
                ],
            ),
            include(
                "hazard_guard_dispenser",
                "dispenser.launch.py",
                {"enable_physical_drop": enable_physical_drop},
                condition=IfCondition(use_dispenser),
            ),
            include(
                "hazard_guard_thermal_analysis",
                "thermal_pipeline.launch.py",
                {
                    "use_sim_time": "false",
                    "simulated": "false",
                    "roi_config": LaunchConfiguration("thermal_roi_config"),
                    "baseline_path": LaunchConfiguration(
                        "thermal_baseline_path"
                    ),
                    "baseline_collection_path": LaunchConfiguration(
                        "thermal_baseline_collection_path"
                    ),
                    "baseline_minimum_valid_visits": LaunchConfiguration(
                        "thermal_baseline_minimum_valid_visits"
                    ),
                    "history_path": LaunchConfiguration(
                        "thermal_history_path"
                    ),
                    "air_temperature_topic": LaunchConfiguration(
                        "thermal_air_temperature_topic"
                    ),
                    "oil_temperature_topic": LaunchConfiguration(
                        "thermal_oil_temperature_topic"
                    ),
                    "sensor_timeout_sec": LaunchConfiguration(
                        "thermal_sensor_timeout_sec"
                    ),
                    # AMCL's saved map is authoritative during physical patrol.
                    # Reject odom-scoped ROIs instead of silently analyzing a
                    # drifting facility region after localization corrections.
                    "required_frame_id": "map",
                    "thermal_image_topic": LaunchConfiguration(
                        "thermal_image_topic"
                    ),
                    "thermal_info_topic": LaunchConfiguration(
                        "thermal_info_topic"
                    ),
                    "depth_image_topic": LaunchConfiguration(
                        "thermal_depth_image_topic"
                    ),
                    "depth_info_topic": LaunchConfiguration(
                        "thermal_depth_info_topic"
                    ),
                    "thermal_scale": LaunchConfiguration("thermal_scale"),
                    "thermal_offset_c": LaunchConfiguration(
                        "thermal_offset_c"
                    ),
                    # HP60C and ThermoEye keep independent header clocks. Pair
                    # on local arrival, then publish z-up coordinates for WebUI.
                    "fusion_sync_by_receipt_time": "true",
                    "fusion_output_frame": "map",
                    "fusion_transform_at_latest": "true",
                    "fusion_color_min_c": "10.0",
                    "fusion_color_max_c": "60.0",
                },
                condition=start_thermal_pipeline,
            ),
            Node(
                package="hazard_guard_thermal_analysis",
                executable="frozen_thermal_map",
                name="hazard_guard_frozen_thermal_map",
                output="screen",
                condition=IfCondition(enable_frozen_thermal_map),
                parameters=[
                    {
                        "use_sim_time": False,
                        "map_cloud_path": LaunchConfiguration(
                            "thermal_map_cloud_path"
                        ),
                        "thermal_state_path": LaunchConfiguration(
                            "thermal_map_state_path"
                        ),
                        "dynamic_state_path": LaunchConfiguration(
                            "thermal_dynamic_state_path"
                        ),
                        "session_id": LaunchConfiguration(
                            "thermal_map_session_id"
                        ),
                        "map_frame": "map",
                        "base_frame": "base_footprint",
                        "sensor_frame": "thermal_camera_optical_frame",
                        "geometry_voxel_size_m": 0.03,
                        "maximum_geometry_voxels": 250000,
                        "maximum_source_vertices": 1000000,
                        "maximum_published_voxels": 100000,
                        "maximum_dynamic_published_voxels": 30000,
                        "association_radius_m": 0.08,
                        "maximum_surface_range_residual_m": 0.05,
                        "dynamic_voxel_size_m": 0.05,
                        "dynamic_minimum_component_voxels": 8,
                        "dynamic_minimum_hits": 2,
                        "dynamic_maximum_misses": 3,
                        "dynamic_maximum_voxels": 50000,
                        "dynamic_visibility_angular_resolution_deg": 1.0,
                        "dynamic_visibility_range_tolerance_m": 0.08,
                        "minimum_match_ratio": 0.30,
                        "keyframe_translation_m": 0.10,
                        "keyframe_rotation_deg": 6.0,
                        "stationary_refresh_interval_sec": 5.0,
                        "rejected_frame_retry_sec": 2.0,
                        "localization_stable_samples": 3,
                        "localization_stable_translation_m": 0.05,
                        "localization_stable_rotation_deg": 3.0,
                        "enable_local_alignment": False,
                    }
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="hazard_guard_mock_robot",
                        executable="initial_pose_once",
                        name="hazard_guard_initial_pose",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": False,
                                "x": ParameterValue(
                                    initial_pose_x,
                                    value_type=float,
                                ),
                                "y": ParameterValue(
                                    initial_pose_y,
                                    value_type=float,
                                ),
                                "yaw": ParameterValue(
                                    initial_pose_yaw,
                                    value_type=float,
                                ),
                                # Span lifecycle activation without repeatedly
                                # resetting localization after patrol is ready.
                                "repeat_count": 3,
                                "interval_sec": 1.0,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
