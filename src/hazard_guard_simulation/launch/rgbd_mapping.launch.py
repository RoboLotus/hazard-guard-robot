"""Saved-map localization plus second-pass RGB-D collection in simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )

    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "gui",
            "world",
            "world_name",
            "map",
            "spawn_x",
            "spawn_y",
            "spawn_z",
            "spawn_yaw",
            "simulation_mode",
            "start_simulation",
            "initial_pose_x",
            "initial_pose_y",
            "initial_pose_yaw",
            "rtabmap_database_path",
            "rtabmap_reset_database",
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("world"),
            DeclareLaunchArgument("world_name"),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_database_path"),
            DeclareLaunchArgument("rtabmap_reset_database", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulation_share / "launch" / "localization.launch.py")
                ),
                launch_arguments={
                    **forwarded,
                    "enable_rgbd_mapping": "true",
                    "auto_initial_pose": "true",
                    "use_person_safety": "false",
                }.items(),
            ),
        ]
    )
