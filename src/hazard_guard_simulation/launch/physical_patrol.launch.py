"""WebUI-managed physical M1 localization and Nav2 patrol stack."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(
    package: str,
    filename: str,
    arguments: Mapping[str, Any] | None = None,
    *,
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
    map_path = LaunchConfiguration("map")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "enable_thermal_pipeline", default_value="false"
            ),
            DeclareLaunchArgument("thermal_roi_config", default_value=""),
            DeclareLaunchArgument(
                "thermal_history_path",
                default_value="~/.local/share/hazard_guard/thermal_history.jsonl",
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
            include("yahboomcar_nav", "laser_bringup_launch.py"),
            include(
                "yahboomcar_nav",
                "navigation_dwa_launch.py",
                {"use_sim_time": "false", "map": map_path},
            ),
            include(
                "hazard_guard_thermal_analysis",
                "thermal_pipeline.launch.py",
                {
                    "use_sim_time": "false",
                    "simulated": "false",
                    "roi_config": LaunchConfiguration("thermal_roi_config"),
                    "history_path": LaunchConfiguration(
                        "thermal_history_path"
                    ),
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
                },
                condition=IfCondition(
                    LaunchConfiguration("enable_thermal_pipeline")
                ),
            ),
            Node(
                package="hazard_guard_mission_manager",
                executable="mission_manager",
                name="hazard_guard_mission_manager",
                output="screen",
            ),
            TimerAction(
                period=8.0,
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
                                "repeat_count": 3,
                                "interval_sec": 0.5,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
