from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _rtabmap_node(
    parameters: Path | LaunchConfiguration,
    *,
    reset_database: bool,
) -> Node:
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            ParameterFile(parameters, allow_substs=True),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "database_path": LaunchConfiguration("database_path"),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("publish_tf"), value_type=bool
                ),
                "map_frame_id": LaunchConfiguration("map_frame_id"),
            },
        ],
        remappings=[
            ("rgbd_image", "rgbd_image"),
            ("scan", "/scan"),
            ("odom", "/odom"),
            ("map", LaunchConfiguration("map_topic")),
        ],
        arguments=["-d"] if reset_database else [],
        condition=(
            IfCondition(LaunchConfiguration("reset_database"))
            if reset_database
            else UnlessCondition(LaunchConfiguration("reset_database"))
        ),
    )


def _color_cloud_assembler_node() -> Node:
    return Node(
        package="rtabmap_util",
        executable="point_cloud_assembler",
        namespace="rtabmap",
        name="color_cloud_assembler",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "fixed_frame_id": "map",
                "frame_id": "map",
                "max_clouds": 220,
                "circular_buffer": True,
                "linear_update": 0.05,
                "angular_update": 0.08,
                "voxel_size": 0.025,
                "range_min": 0.2,
                "range_max": 4.0,
                "wait_for_transform": 0.5,
                "qos": 2,
            }
        ],
        remappings=[
            ("cloud", "/hazard_guard/rtabmap/cloud_frame"),
            (
                "assembled_cloud",
                "/hazard_guard/rtabmap/cloud_surface",
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration("optimized_cloud")),
    )


def _optimized_map_assembler_node() -> Node:
    """Rebuild the public cloud whenever RTAB-Map optimizes node poses."""

    return Node(
        package="rtabmap_util",
        executable="map_assembler",
        namespace="rtabmap",
        name="optimized_map_assembler",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_always_update": True,
                "map_cleanup": True,
                "cloud_output_voxelized": True,
                "Grid/3D": "true",
                "Grid/RangeMin": "0.2",
                "Grid/RangeMax": "4.0",
                # Match the field-tested physical visualization policy.
                # RTAB-Map parameters are expressed in metres.
                "Grid/CellSize": "0.03",
            }
        ],
        remappings=[
            (
                "cloud_map",
                "/hazard_guard/rtabmap/cloud_surface_optimized",
            )
        ],
        condition=IfCondition(LaunchConfiguration("optimized_cloud")),
    )


def _optimized_cloud_guard_node() -> Node:
    """Bound the large optimized snapshot before WebUI DDS transport."""

    return Node(
        package="hazard_guard_simulation",
        executable="adaptive_cloud_guard.py",
        name="adaptive_cloud_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "normal_points": 9000,
                "high_load_points": 4500,
                "normal_surface_hz": 1.0,
                "high_load_surface_hz": 0.5,
            }
        ],
        remappings=[
            ("input", "/hazard_guard/rtabmap/cloud_frame_guard_unused"),
            (
                "surface_input",
                "/hazard_guard/rtabmap/cloud_surface_optimized",
            ),
            ("surface_output", "/hazard_guard/rtabmap/cloud_surface"),
            (
                "surface_compat_output",
                "/hazard_guard/rtabmap/cloud_frame_raw",
            ),
            ("status", "/hazard_guard/rtabmap/cloud_guard/status"),
        ],
        condition=IfCondition(LaunchConfiguration("optimized_cloud")),
    )


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    parameters = simulation_share / "config" / "rtabmap_sim.yaml"
    rviz_config = simulation_share / "rviz" / "rtabmap_sim.rviz"

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
    start_rviz = LaunchConfiguration("rviz")
    start_demo_route = LaunchConfiguration("demo_route")
    parameters_file = LaunchConfiguration("parameters_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    simulation_share / "worlds" / "demo_facility.sdf"
                ),
            ),
            DeclareLaunchArgument("world_name", default_value="demo_facility"),
            DeclareLaunchArgument("spawn_x", default_value="0.13"),
            DeclareLaunchArgument("spawn_y", default_value="-0.99"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("visualize_sensors", default_value="false"),
            DeclareLaunchArgument("include_dispenser", default_value="true"),
            DeclareLaunchArgument("dispenser_mass", default_value="1.2"),
            DeclareLaunchArgument("heat_source_profile", default_value=""),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "parameters_file",
                default_value=str(parameters),
                description=(
                    "RTAB-Map parameter profile. The two-pass workflow uses "
                    "rtabmap_rgbd_capture.yaml so external odometry remains "
                    "authoritative and RTAB-Map only records RGB-D data."
                ),
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description=(
                    "Publish RTAB-Map's map-to-odom TF. Disable when SLAM "
                    "Toolbox is the navigation-frame authority."
                ),
            ),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "thermal_cloud",
                default_value="true",
                description=(
                    "Build the thermal 3D map by projecting depth into the "
                    "calibrated thermal frame"
                ),
            ),
            DeclareLaunchArgument(
                "min_temp_c",
                default_value="10.0",
                description="Blue end of the thermal map's colour window",
            ),
            DeclareLaunchArgument(
                "max_temp_c",
                default_value="60.0",
                description="Red end of the thermal map's colour window",
            ),
            DeclareLaunchArgument(
                "optimized_cloud",
                default_value="false",
                description=(
                    "Publish the graph-optimized RTAB-Map cloud instead of "
                    "the irreversible raw point-cloud assembly"
                ),
            ),
            DeclareLaunchArgument(
                "demo_route",
                default_value="false",
                description="Run the bounded simulation-only RGB-D scan route",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/hazard_guard_rtabmap_sim.db",
                description="Simulation-only RTAB-Map database path",
            ),
            DeclareLaunchArgument(
                "reset_database",
                default_value="true",
                description="Reset only the selected simulation database on start",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulation_share / "launch" / "simulation.launch.py")
                ),
                launch_arguments={
                    "gui": gui,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
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
                }.items(),
                condition=IfCondition(start_simulation),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="rtabmap_sync",
                        executable="rgbd_sync",
                        namespace="rtabmap",
                        name="rgbd_sync",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                ),
                                "approx_sync": True,
                                "approx_sync_max_interval": 0.08,
                                "sync_queue_size": 30,
                                "qos": 2,
                            }
                        ],
                        remappings=[
                            ("rgb/image", "/camera/image_raw"),
                            ("rgb/camera_info", "/camera/camera_info"),
                            ("depth/image", "/depth_camera/image_raw"),
                        ],
                    ),
                    Node(
                        package="rtabmap_util",
                        executable="point_cloud_xyzrgb",
                        namespace="rtabmap",
                        name="color_cloud_frame",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                ),
                                "approx_sync": True,
                                "approx_sync_max_interval": 0.08,
                                "sync_queue_size": 20,
                                "topic_queue_size": 5,
                                "qos": 2,
                                "qos_camera_info": 2,
                                "decimation": 4,
                                "min_depth": 0.2,
                                "max_depth": 4.0,
                                "filter_nans": True,
                            }
                        ],
                        remappings=[
                            ("rgb/image", "/camera/image_raw"),
                            ("depth/image", "/depth_camera/image_raw"),
                            (
                                "rgb/camera_info",
                                "/camera/camera_info",
                            ),
                            (
                                "cloud",
                                "/hazard_guard/rtabmap/cloud_frame",
                            ),
                        ],
                        condition=UnlessCondition(
                            LaunchConfiguration("optimized_cloud")
                        ),
                    ),
                    _rtabmap_node(parameters_file, reset_database=True),
                    _rtabmap_node(parameters_file, reset_database=False),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    _color_cloud_assembler_node(),
                    _optimized_map_assembler_node(),
                    _optimized_cloud_guard_node(),
                ],
            ),
            TimerAction(
                # After the assembler, for the same reason: map<-camera TF has
                # to exist before either node can put a point in the map.
                period=7.0,
                actions=[
                    Node(
                        package="hazard_guard_simulation",
                        executable="thermal_cloud.py",
                        name="thermal_cloud",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                ),
                                # "map", not map_frame_id: RTAB-Map's own grid
                                # is namespaced away when SLAM Toolbox owns the
                                # navigation frame, and the colour assembler
                                # above publishes into "map" for the same
                                # reason - both clouds have to share it.
                                "map_frame": "map",
                                # Launch arguments arrive as strings and the
                                # node declares these as double.
                                "min_temp_c": ParameterValue(
                                    LaunchConfiguration("min_temp_c"),
                                    value_type=float,
                                ),
                                "max_temp_c": ParameterValue(
                                    LaunchConfiguration("max_temp_c"),
                                    value_type=float,
                                ),
                            }
                        ],
                        condition=IfCondition(LaunchConfiguration("thermal_cloud")),
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rtabmap_sim_rviz",
                        output="screen",
                        arguments=["-d", str(rviz_config)],
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                )
                            }
                        ],
                        condition=IfCondition(start_rviz),
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="hazard_guard_simulation",
                        executable="rtabmap_scan_demo.py",
                        name="hazard_guard_rtabmap_scan_demo",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                        condition=IfCondition(start_demo_route),
                    )
                ],
            ),
        ]
    )
