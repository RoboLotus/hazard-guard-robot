from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
    slam_parameters = simulation_share / "config" / "slam.yaml"

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
    slam = LaunchConfiguration("slam")
    use_person_safety = LaunchConfiguration("use_person_safety")
    person_model_path = LaunchConfiguration("person_model_path")
    person_device = LaunchConfiguration("person_device")
    bridge_velocity_topic = PythonExpression(
        [
            "'/cmd_vel_safe' if '",
            use_person_safety,
            "'.lower() == 'true' else '/cmd_vel'",
        ]
    )
    use_thermal_pipeline = LaunchConfiguration("use_thermal_pipeline")
    thermal_history_path = LaunchConfiguration("thermal_history_path")
    enable_rgbd_mapping = LaunchConfiguration("enable_rgbd_mapping")
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    rtabmap_reset_database = LaunchConfiguration("rtabmap_reset_database")
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
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description=(
                    "Keep mapping while patrolling: SLAM Toolbox replaces "
                    "map_server + AMCL, so the map keeps growing and can be "
                    "saved again. The 'map' argument is then unused - "
                    "SLAM Toolbox starts from an empty map."
                ),
            ),
            DeclareLaunchArgument("use_person_safety", default_value="false"),
            DeclareLaunchArgument("person_model_path", default_value="yolo11n.pt"),
            DeclareLaunchArgument("person_device", default_value=""),
            DeclareLaunchArgument("use_thermal_pipeline", default_value="true"),
            DeclareLaunchArgument(
                "thermal_history_path",
                default_value=(
                    "~/.local/share/hazard_guard/thermal_history.jsonl"
                ),
            ),
            DeclareLaunchArgument("enable_rgbd_mapping", default_value="false"),
            DeclareLaunchArgument(
                "rtabmap_database_path",
                default_value="/tmp/hazard_guard_rtabmap_capture.db",
            ),
            DeclareLaunchArgument("rtabmap_reset_database", default_value="false"),
            DeclareLaunchArgument("use_performance_monitor", default_value="true"),
            DeclareLaunchArgument("performance_storage_path", default_value=""),
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
                    "cmd_vel_ros_topic": bridge_velocity_topic,
                    "use_thermal_pipeline": use_thermal_pipeline,
                    "thermal_history_path": thermal_history_path,
                }.items(),
                condition=IfCondition(start_simulation),
            ),
            TimerAction(
                period=4.0,
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
                        condition=UnlessCondition(slam),
                    )
                ],
            ),
            # slam:=true patrol. The same SLAM Toolbox node the mapping mode
            # uses, so /map keeps updating and map_saver_cli can store it, with
            # Nav2 on top of it. nav2_bringup's own slam path is not used: it
            # starts the sync node with its own parameters.
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[str(slam_parameters)],
                        condition=IfCondition(slam),
                    )
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
                        condition=IfCondition(slam),
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
                        parameters=[
                            {
                                "use_sim_time": True,
                                "safety_supervision_enabled": ParameterValue(
                                    use_person_safety,
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        simulation_share
                        / "launch"
                        / "rgbd_capture_after_localization.launch.py"
                    )
                ),
                launch_arguments={
                    "backend": "sim",
                    "use_sim_time": "true",
                    "map": map_file,
                    "database_path": rtabmap_database_path,
                    "reset_database": rtabmap_reset_database,
                    "readiness_timeout_sec": "45.0",
                }.items(),
                condition=IfCondition(enable_rgbd_mapping),
            ),
        ]
    )
