"""WebUI-managed physical mapping for 2D or 2D + RGB-D sessions."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments=None, *, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    workspace = Path(
        os.getenv("HAZARD_GUARD_WORKSPACE", os.getcwd())
    ).expanduser().resolve()
    database_path = LaunchConfiguration("database_path")
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value=str(workspace / "runtime" / "maps" / "physical_rtabmap.db"),
        ),
        DeclareLaunchArgument("enable_rtabmap", default_value="true"),
        # The vendor top-level launch does not forward this argument, while its
        # nested SLAM launch reads the global configuration directly.
        SetLaunchConfiguration("use_sim_time", "false"),
        include("yahboomcar_nav", "map_slam_toolbox_launch.py"),
        include(
            "ascamera",
            "hp60c.launch.py",
            condition=IfCondition(enable_rtabmap),
        ),
        include(
            "hazard_guard_simulation",
            "rtabmap_real.launch.py",
            {"database_path": database_path},
            condition=IfCondition(enable_rtabmap),
        ),
    ])
