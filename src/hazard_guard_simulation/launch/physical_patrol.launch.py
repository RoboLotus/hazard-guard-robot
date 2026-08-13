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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(
    package: str,
    filename: str,
    arguments: Mapping[str, Any] | None = None,
) -> IncludeLaunchDescription:
    package_share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / filename)
        ),
        launch_arguments=(arguments or {}).items(),
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
            SetLaunchConfiguration("use_sim_time", "false"),
            include("yahboomcar_nav", "laser_bringup_launch.py"),
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
            Node(
                package="hazard_guard_mission_manager",
                executable="mission_manager",
                name="hazard_guard_mission_manager",
                output="screen",
                parameters=[nav2_params_file],
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
