from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("profile", default_value="navigation-core"),
        DeclareLaunchArgument("storage_root", default_value="~/.local/share/hazard_guard/bags"),
        DeclareLaunchArgument("session_name", default_value="field-session"),
        DeclareLaunchArgument("storage_id", default_value="sqlite3"),
        DeclareLaunchArgument("minimum_free_gb", default_value="2.0"),
        DeclareLaunchArgument("max_duration_seconds", default_value="1800.0"),
        DeclareLaunchArgument("max_size_gb", default_value="10.0"),
        DeclareLaunchArgument("allow_experimental", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("enable_control_services", default_value="false"),
    ]
    return LaunchDescription(arguments + [
        Node(
            package="hazard_guard_bag_recorder",
            executable="bag_session_manager",
            name="hazard_guard_bag_session_manager",
            output="screen",
            parameters=[{
                "profile": LaunchConfiguration("profile"),
                "storage_root": LaunchConfiguration("storage_root"),
                "session_name": LaunchConfiguration("session_name"),
                "storage_id": LaunchConfiguration("storage_id"),
                "minimum_free_gb": LaunchConfiguration("minimum_free_gb"),
                "max_duration_seconds": LaunchConfiguration("max_duration_seconds"),
                "max_size_gb": LaunchConfiguration("max_size_gb"),
                "allow_experimental": LaunchConfiguration("allow_experimental"),
                "auto_start": LaunchConfiguration("auto_start"),
                "enable_control_services": LaunchConfiguration("enable_control_services"),
            }],
        )
    ])
