from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("hazard_guard_safety_supervisor")
    config_file = os.path.join(package_share, "config", "person_safety.yaml")

    return LaunchDescription(
        [
            Node(
                package="hazard_guard_safety_supervisor",
                executable="person_safety_supervisor",
                name="person_safety_supervisor",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
