from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(get_package_share_directory("hazard_guard_simulation"))
    parameters = simulation_share / "config" / "slam.yaml"
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    simulation_mode = LaunchConfiguration("simulation_mode")
    visualize_sensors = LaunchConfiguration("visualize_sensors")
    include_dispenser = LaunchConfiguration("include_dispenser")
    dispenser_mass = LaunchConfiguration("dispenser_mass")
    start_simulation = LaunchConfiguration("start_simulation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    simulation_share / "worlds" / "demo_facility.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="demo_facility",
                description=(
                    "The <world name> value inside the selected SDF file"
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="-1.15"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("visualize_sensors", default_value="false"),
            DeclareLaunchArgument("include_dispenser", default_value="true"),
            DeclareLaunchArgument("dispenser_mass", default_value="1.2"),
            DeclareLaunchArgument(
                "start_simulation",
                default_value="true",
                description=(
                    "Start Gazebo and the simulated robot. Set false when a "
                    "singleton simulator is supervised separately."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulation_share / "launch" / "simulation.launch.py")
                ),
                launch_arguments={
                    "gui": gui,
                    "use_sim_time": "true",
                    "world": world,
                    "world_name": world_name,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_yaw": spawn_yaw,
                    "simulation_mode": simulation_mode,
                    "visualize_sensors": visualize_sensors,
                    "include_dispenser": include_dispenser,
                    "dispenser_mass": dispenser_mass,
                }.items(),
                condition=IfCondition(start_simulation),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[str(parameters)],
                    )
                ],
            ),
        ]
    )
