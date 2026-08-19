from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("storage_path", default_value=""),
            DeclareLaunchArgument("sample_interval_sec", default_value="1.0"),
            Node(
                package="hazard_guard_performance_monitor",
                executable="performance_monitor",
                name="hazard_guard_performance_monitor",
                output="screen",
                parameters=[
                    {
                        "storage_path": LaunchConfiguration("storage_path"),
                        "sample_interval_sec": LaunchConfiguration(
                            "sample_interval_sec"
                        ),
                    }
                ],
            ),
        ]
    )
