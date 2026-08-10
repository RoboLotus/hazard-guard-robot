"""WebUI-managed physical mapping: M1/LiDAR 2D SLAM + HP60C + RTAB-Map."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def include(package, filename, arguments=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", filename)), launch_arguments=(arguments or {}).items())

def generate_launch_description():
    database_path = LaunchConfiguration("database_path")
    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value=os.path.expanduser("~/RoboLotus/hazard-guard-robot/runtime/maps/physical_rtabmap.db")),
        # The vendor top-level launch does not forward this argument, while its
        # nested SLAM launch reads the global configuration directly.
        SetLaunchConfiguration("use_sim_time", "false"),
        include("yahboomcar_nav", "map_slam_toolbox_launch.py"),
        include("ascamera", "hp60c.launch.py"),
        include("hazard_guard_simulation", "rtabmap_real.launch.py", {"database_path": database_path}),
    ])
