from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(get_package_share_directory("hazard_guard_simulation"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    detection_share = Path(
        get_package_share_directory("hazard_guard_person_detection")
    )
    safety_share = Path(
        get_package_share_directory("hazard_guard_safety_supervisor")
    )
    nav2_parameters = simulation_share / "config" / "nav2.yaml"
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
    use_person_safety = LaunchConfiguration("use_person_safety")
    enable_hazard_approval = LaunchConfiguration("enable_hazard_approval")
    person_model_path = LaunchConfiguration("person_model_path")
    person_device = LaunchConfiguration("person_device")
    bridge_velocity_topic = PythonExpression(
        [
            "'/cmd_vel_safe' if '",
            use_person_safety,
            "'.lower() == 'true' else '/cmd_vel'",
        ]
    )
    heat_source_profile = LaunchConfiguration("heat_source_profile")
    use_thermal_pipeline = LaunchConfiguration("use_thermal_pipeline")
    thermal_history_path = LaunchConfiguration("thermal_history_path")
    use_performance_monitor = LaunchConfiguration("use_performance_monitor")
    performance_storage_path = LaunchConfiguration("performance_storage_path")

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
            DeclareLaunchArgument("use_person_safety", default_value="false"),
            DeclareLaunchArgument(
                "enable_hazard_approval",
                default_value="false",
                description=(
                    "Pause simulated patrols when a correlated thermal "
                    "warning or critical result needs administrator approval."
                ),
            ),
            DeclareLaunchArgument("person_model_path", default_value="yolo11n.pt"),
            DeclareLaunchArgument("person_device", default_value=""),
            DeclareLaunchArgument("heat_source_profile", default_value=""),
            DeclareLaunchArgument("use_thermal_pipeline", default_value="true"),
            DeclareLaunchArgument("thermal_history_path", default_value="~/.local/share/hazard_guard/thermal_history.jsonl"),
            DeclareLaunchArgument("use_performance_monitor", default_value="true"),
            DeclareLaunchArgument("performance_storage_path", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulation_share / "launch" / "slam.launch.py")
                ),
                launch_arguments={
                    "gui": gui,
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
                    "cmd_vel_ros_topic": bridge_velocity_topic,
                    "heat_source_profile": heat_source_profile,
                    "use_thermal_pipeline": use_thermal_pipeline,
                    "thermal_history_path": thermal_history_path,
                }.items(),
            ),
            TimerAction(
                period=6.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                detection_share
                                / "launch"
                                / "person_detection.launch.py"
                            )
                        ),
                        launch_arguments={
                            "rgb_topic": "/camera/image_raw",
                            "depth_topic": "/depth_camera/image_raw",
                            "model_path": person_model_path,
                            "device": person_device,
                            "simulated": "true",
                            "depth_registration_verified": "true",
                            "use_sim_time": "true",
                        }.items(),
                        condition=IfCondition(use_person_safety),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                safety_share
                                / "launch"
                                / "person_safety.launch.py"
                            )
                        ),
                        launch_arguments={"use_sim_time": "true"}.items(),
                        condition=IfCondition(use_person_safety),
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(nav2_share / "launch" / "navigation_launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            "autostart": "true",
                            "use_composition": "False",
                            "params_file": str(nav2_parameters),
                        }.items(),
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
                        parameters=[
                            {
                                "use_sim_time": True,
                                "safety_supervision_enabled": ParameterValue(
                                    use_person_safety,
                                    value_type=bool,
                                ),
                                "hazard_approval_enabled": ParameterValue(
                                    enable_hazard_approval,
                                    value_type=bool,
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="hazard_guard_performance_monitor",
                        executable="performance_monitor",
                        name="hazard_guard_performance_monitor",
                        output="screen",
                        condition=IfCondition(use_performance_monitor),
                        parameters=[
                            {
                                "storage_path": performance_storage_path,
                                "sample_interval_sec": 1.0,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
