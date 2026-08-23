"""Sensor-agnostic thermal point-cloud and voxel analysis pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("hazard_guard_thermal_analysis"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("simulated", default_value="true"),
            DeclareLaunchArgument("roi_config", default_value=str(share / "config" / "demo_facility_scaled_rois.json")),
            DeclareLaunchArgument("baseline_path", default_value=""),
            DeclareLaunchArgument("baseline_collection_path", default_value=""),
            DeclareLaunchArgument("baseline_minimum_valid_visits", default_value="10"),
            DeclareLaunchArgument("history_path", default_value="~/.local/share/hazard_guard/thermal_history.jsonl"),
            DeclareLaunchArgument("air_temperature_topic", default_value=""),
            DeclareLaunchArgument("oil_temperature_topic", default_value=""),
            DeclareLaunchArgument("sensor_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("required_frame_id", default_value=""),
            DeclareLaunchArgument("thermal_image_topic", default_value="/thermal_camera/image_raw"),
            DeclareLaunchArgument("thermal_info_topic", default_value="/thermal_camera/camera_info"),
            DeclareLaunchArgument("depth_image_topic", default_value="/depth_camera/image_raw"),
            DeclareLaunchArgument("depth_info_topic", default_value="/depth_camera/camera_info"),
            DeclareLaunchArgument("thermal_scale", default_value="0.01"),
            DeclareLaunchArgument("thermal_offset_c", default_value="-273.15"),
            DeclareLaunchArgument("fusion_stride", default_value="4"),
            DeclareLaunchArgument("publish_detections", default_value="true"),
            Node(
                package="hazard_guard_thermal_analysis",
                executable="thermal_depth_fusion",
                name="hazard_guard_thermal_depth_fusion",
                output="screen",
                parameters=[{
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "thermal_scale": ParameterValue(LaunchConfiguration("thermal_scale"), value_type=float),
                    "thermal_offset_c": ParameterValue(LaunchConfiguration("thermal_offset_c"), value_type=float),
                    "stride": ParameterValue(LaunchConfiguration("fusion_stride"), value_type=int),
                }],
                remappings=[
                    ("/hazard_guard/thermal/image", LaunchConfiguration("thermal_image_topic")),
                    ("/hazard_guard/thermal/camera_info", LaunchConfiguration("thermal_info_topic")),
                    ("/hazard_guard/depth/image", LaunchConfiguration("depth_image_topic")),
                    ("/hazard_guard/depth/camera_info", LaunchConfiguration("depth_info_topic")),
                ],
            ),
            Node(
                package="hazard_guard_thermal_analysis",
                executable="thermal_voxel_analyzer",
                name="hazard_guard_thermal_voxel_analyzer",
                output="screen",
                parameters=[{
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "roi_config": LaunchConfiguration("roi_config"),
                    "baseline_path": LaunchConfiguration("baseline_path"),
                    "baseline_collection_path": LaunchConfiguration("baseline_collection_path"),
                    "baseline_minimum_valid_visits": ParameterValue(
                        LaunchConfiguration("baseline_minimum_valid_visits"), value_type=int
                    ),
                    "history_path": LaunchConfiguration("history_path"),
                    "air_temperature_topic": LaunchConfiguration("air_temperature_topic"),
                    "oil_temperature_topic": LaunchConfiguration("oil_temperature_topic"),
                    "sensor_timeout_sec": ParameterValue(
                        LaunchConfiguration("sensor_timeout_sec"), value_type=float
                    ),
                    "required_frame_id": LaunchConfiguration("required_frame_id"),
                    "publish_detections": ParameterValue(LaunchConfiguration("publish_detections"), value_type=bool),
                    "simulated": ParameterValue(LaunchConfiguration("simulated"), value_type=bool),
                }],
            ),
        ]
    )
