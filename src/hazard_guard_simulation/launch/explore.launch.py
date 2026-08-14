"""Add Nav2 and frontier exploration to a mapping session already running.

The mapping stack - Gazebo, SLAM Toolbox, RTAB-Map - is started elsewhere,
by `slam.launch.py` or by the WebUI's 지도 운용 모드. This adds the two pieces
that mapping does not include: a navigation stack to drive with, and the node
that decides where to drive.

Nav2 comes from nav2_bringup's navigation_launch.py, which is the bringup
without map_server or AMCL - SLAM Toolbox is already publishing /map and
map->odom, and a second publisher of either would fight it.

    ros2 launch hazard_guard_simulation explore.launch.py
    ros2 launch hazard_guard_simulation explore.launch.py nav2:=false
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    nav2_parameters = simulation_share / "config" / "nav2.yaml"

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "nav2",
                default_value="true",
                description=(
                    "Start Nav2. Set false when a patrol stack is already "
                    "providing navigate_to_pose."
                ),
            ),
            DeclareLaunchArgument(
                "minimum_frontier_cells",
                default_value="6",
                description="Ignore frontier groups smaller than this",
            ),
            DeclareLaunchArgument(
                "goal_timeout_sec",
                default_value="90.0",
                description="Give up on one frontier after this long",
            ),
            DeclareLaunchArgument(
                "run_timeout_sec",
                default_value="1200.0",
                description="Hard stop for the whole exploration run",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": str(nav2_parameters),
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
                condition=IfCondition(LaunchConfiguration("nav2")),
            ),
            # Nav2's lifecycle managers need to reach active before the first
            # goal, and the costmaps need a map to have arrived. Ten seconds is
            # the difference between a clean start and one rejected goal.
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="hazard_guard_simulation",
                        executable="auto_explore.py",
                        name="hazard_guard_auto_explore",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                # Launch arguments arrive as strings; the node
                                # declares these as int and float.
                                "minimum_frontier_cells": ParameterValue(
                                    LaunchConfiguration(
                                        "minimum_frontier_cells"
                                    ),
                                    value_type=int,
                                ),
                                "goal_timeout_sec": ParameterValue(
                                    LaunchConfiguration("goal_timeout_sec"),
                                    value_type=float,
                                ),
                                "run_timeout_sec": ParameterValue(
                                    LaunchConfiguration("run_timeout_sec"),
                                    value_type=float,
                                ),
                            }
                        ],
                    )
                ],
            ),
        ]
    )
