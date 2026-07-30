from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("hazard_guard_bringup"))
    parameters = package_share / "config" / "mock_robot.yaml"

    return LaunchDescription(
        [
            Node(
                package="hazard_guard_mock_robot",
                executable="mock_robot",
                name="hazard_guard_mock_robot",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
