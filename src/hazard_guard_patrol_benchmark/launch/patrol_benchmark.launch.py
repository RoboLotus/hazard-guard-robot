from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("hazard_guard_patrol_benchmark"))
    defaults = str(share / "config" / "benchmark_defaults.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_id", default_value="real_factory"),
            DeclareLaunchArgument("simulation_env_path", default_value=""),
            DeclareLaunchArgument("storage_path", default_value=""),
            DeclareLaunchArgument("ground_truth_topic", default_value="/odom"),
            DeclareLaunchArgument("compare_localization", default_value="false"),
            Node(
                package="hazard_guard_patrol_benchmark",
                executable="patrol_benchmark",
                name="hazard_guard_patrol_benchmark",
                output="screen",
                parameters=[
                    defaults,
                    {
                        "world_id": LaunchConfiguration("world_id"),
                        "simulation_env_path": LaunchConfiguration(
                            "simulation_env_path"
                        ),
                        "storage_path": LaunchConfiguration("storage_path"),
                        "ground_truth_topic": LaunchConfiguration(
                            "ground_truth_topic"
                        ),
                        "compare_localization": LaunchConfiguration(
                            "compare_localization"
                        ),
                    },
                ],
            ),
        ]
    )
