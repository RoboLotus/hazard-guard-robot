from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("hazard_guard_dispenser"),
                        "config",
                        "dispenser_physical.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "enable_physical_drop",
                default_value="false",
                description=(
                    "Enable only after the stopped-robot, BLE and physical "
                    "dispenser checklist has passed."
                ),
            ),
            Node(
                package="hazard_guard_dispenser",
                executable="dispenser_node",
                name="dispenser_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "enable_physical_drop": ParameterValue(
                            LaunchConfiguration("enable_physical_drop"),
                            value_type=bool,
                        )
                    },
                ],
            ),
        ]
    )
