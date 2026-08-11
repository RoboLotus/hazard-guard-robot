"""WebUI-managed physical M1 localization and Nav2 patrol stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    map_path = LaunchConfiguration("map")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        SetLaunchConfiguration("use_sim_time", "false"),
        include("yahboomcar_nav", "laser_bringup_launch.py"),
        include("yahboomcar_nav", "navigation_dwa_launch.py", {"use_sim_time": "false", "map": map_path}),
        Node(package="hazard_guard_mission_manager", executable="mission_manager", name="hazard_guard_mission_manager", output="screen"),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="hazard_guard_mock_robot",
                    executable="initial_pose_once",
                    name="hazard_guard_initial_pose",
                    output="screen",
                    parameters=[{
                        "use_sim_time": False,
                        "x": ParameterValue(initial_pose_x, value_type=float),
                        "y": ParameterValue(initial_pose_y, value_type=float),
                        "yaw": ParameterValue(initial_pose_yaw, value_type=float),
                        "repeat_count": 3,
                        "interval_sec": 0.5,
                    }],
                )
            ],
        ),
    ])
