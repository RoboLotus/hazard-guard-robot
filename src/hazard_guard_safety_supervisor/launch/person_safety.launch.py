from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("hazard_guard_safety_supervisor")
    config_file = os.path.join(package_share, "config", "person_safety.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="hazard_guard_safety_supervisor",
                executable="person_safety_supervisor",
                name="person_safety_supervisor",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="hazard_guard_safety_supervisor",
                executable="cmd_vel_safety_gate",
                name="cmd_vel_safety_gate",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        )
                    },
                ],
            ),
        ]
    )
