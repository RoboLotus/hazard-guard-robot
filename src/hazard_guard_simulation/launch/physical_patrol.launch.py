"""WebUI-managed physical M1 localization and Nav2 patrol stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def include(package, filename, arguments=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", filename)), launch_arguments=(arguments or {}).items())

def generate_launch_description():
    map_path = LaunchConfiguration("map")
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        include("yahboomcar_nav", "laser_bringup_launch.py"),
        include("yahboomcar_nav", "navigation_dwa_launch.py", {"use_sim_time": "false", "map": map_path}),
        Node(package="hazard_guard_mission_manager", executable="mission_manager", name="hazard_guard_mission_manager", output="screen"),
    ])
