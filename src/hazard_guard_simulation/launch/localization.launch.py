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
    simulation_share = Path(get_package_share_directory("hazard_guard_simulation"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    nav2_parameters = simulation_share / "config" / "nav2.yaml"

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    map_file = LaunchConfiguration("map")
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
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
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
                "map",
                description="Absolute path to a saved occupancy map YAML file.",
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
            DeclareLaunchArgument(
                "start_simulation",
                default_value="true",
                description=(
                    "Start Gazebo and the simulated robot. Set false when a "
                    "singleton simulator is supervised separately."
                ),
            ),
            DeclareLaunchArgument(
                "auto_initial_pose",
                default_value="true",
                description="Publish the Gazebo spawn pose to AMCL after Nav2 starts.",
            ),
            DeclareLaunchArgument("initial_pose_delay", default_value="12.0"),
            DeclareLaunchArgument("initial_pose_x", default_value=spawn_x),
            DeclareLaunchArgument("initial_pose_y", default_value=spawn_y),
            DeclareLaunchArgument("initial_pose_yaw", default_value=spawn_yaw),
            DeclareLaunchArgument("use_thermal_pipeline", default_value="true"),
            DeclareLaunchArgument("thermal_history_path", default_value="/tmp/hazard_guard_thermal_history.jsonl"),
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
                    "use_thermal_pipeline": use_thermal_pipeline,
                    "thermal_history_path": thermal_history_path,
                }.items(),
                condition=IfCondition(start_simulation),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(nav2_share / "launch" / "bringup_launch.py")
                        ),
                        launch_arguments={
                            "map": map_file,
                            "slam": "False",
                            "use_sim_time": "true",
                            "autostart": "true",
                            "use_composition": "False",
                            "params_file": str(nav2_parameters),
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=initial_pose_delay,
                actions=[
                    Node(
                        package="hazard_guard_mock_robot",
                        executable="initial_pose_once",
                        name="hazard_guard_initial_pose",
                        output="screen",
                        condition=IfCondition(auto_initial_pose),
                        parameters=[
                            {
                                "use_sim_time": True,
                                "x": ParameterValue(initial_pose_x, value_type=float),
                                "y": ParameterValue(initial_pose_y, value_type=float),
                                "yaw": ParameterValue(initial_pose_yaw, value_type=float),
                                # AMCL is active before this delayed node starts.
                                # A short discovery burst is sufficient; a long
                                # burst would reset localization after Nav2 begins.
                                "repeat_count": 3,
                                "interval_sec": 0.5,
                            }
                        ],
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="hazard_guard_mission_manager",
                        executable="mission_manager",
                        name="hazard_guard_mission_manager",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                    )
                ],
            ),
        ]
    )
