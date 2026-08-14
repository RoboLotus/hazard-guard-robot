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
    heat_source_profile = LaunchConfiguration("heat_source_profile")
    start_simulation = LaunchConfiguration("start_simulation")
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    cmd_vel_ros_topic = LaunchConfiguration("cmd_vel_ros_topic")
    use_thermal_pipeline = LaunchConfiguration("use_thermal_pipeline")
    thermal_history_path = LaunchConfiguration("thermal_history_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    simulation_share / "worlds" / "demo_facility_scaled.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="demo_facility_scaled",
                description=(
                    "The <world name> value inside the selected SDF file"
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0975"),
            DeclareLaunchArgument("spawn_y", default_value="-1.4121"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("visualize_sensors", default_value="false"),
            DeclareLaunchArgument("include_dispenser", default_value="true"),
            DeclareLaunchArgument("dispenser_mass", default_value="1.2"),
            DeclareLaunchArgument("heat_source_profile", default_value=""),
            DeclareLaunchArgument("cmd_vel_ros_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "start_simulation",
                default_value="true",
                description=(
                    "Start Gazebo and the simulated robot. Set false when a "
                    "singleton simulator is supervised separately."
                ),
            ),
            DeclareLaunchArgument(
                "enable_rtabmap",
                default_value="false",
                description="Add RGB-D RTAB-Map collection to 2D SLAM Toolbox.",
            ),
            DeclareLaunchArgument(
                "rtabmap_database_path",
                default_value="/tmp/hazard_guard_rtabmap_sim.db",
            ),
            DeclareLaunchArgument("use_thermal_pipeline", default_value="true"),
            DeclareLaunchArgument("thermal_history_path", default_value="~/.local/share/hazard_guard/thermal_history.jsonl"),
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
                    "heat_source_profile": heat_source_profile,
                    "cmd_vel_ros_topic": cmd_vel_ros_topic,
                    "use_thermal_pipeline": use_thermal_pipeline,
                    "thermal_history_path": thermal_history_path,
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulation_share / "launch" / "rtabmap_sim.launch.py")
                ),
                launch_arguments={
                    "gui": "false",
                    "use_sim_time": "true",
                    "start_simulation": "false",
                    "rviz": "false",
                    "demo_route": "false",
                    "database_path": rtabmap_database_path,
                    "reset_database": "true",
                    # SLAM Toolbox remains the only map->odom publisher used
                    # by Nav2. RTAB-Map keeps its grid and graph namespaced.
                    "publish_tf": "false",
                    "map_frame_id": "rtabmap_map",
                    "map_topic": "/rtabmap/grid_map",
                }.items(),
                condition=IfCondition(enable_rtabmap),
            ),
        ]
    )
