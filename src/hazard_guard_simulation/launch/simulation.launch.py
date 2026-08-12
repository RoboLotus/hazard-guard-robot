import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hazard_guard_sensor_config import TMC160B
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    thermal_analysis_share = Path(
        get_package_share_directory("hazard_guard_thermal_analysis")
    )
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    default_world = simulation_share / "worlds" / "demo_facility_scaled.sdf"
    robot = simulation_share / "urdf" / "hazard_guard_m1.urdf.xacro"

    use_sim_time = LaunchConfiguration("use_sim_time")
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
    use_thermal_pipeline = LaunchConfiguration("use_thermal_pipeline")
    thermal_history_path = LaunchConfiguration("thermal_history_path")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                str(robot),
                " simulation_mode:=",
                simulation_mode,
                " visualize_sensors:=",
                visualize_sensors,
                " include_dispenser:=",
                include_dispenser,
                " dispenser_mass:=",
                dispenser_mass,
                " thermal_width:=",
                str(TMC160B.width),
                " thermal_height:=",
                str(TMC160B.height),
                " thermal_update_rate:=",
                str(TMC160B.frame_rate_hz),
                " thermal_horizontal_fov:=",
                str(TMC160B.horizontal_fov_rad),
                " thermal_clip_near:=",
                str(TMC160B.clip_near_m),
                " thermal_clip_far:=",
                str(TMC160B.visualization_range_m),
            ]
        ),
        value_type=str,
    )

    gazebo_launch = str(ros_gz_share / "launch" / "gz_sim.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=str(default_world),
                description="Absolute path to the Gazebo Fortress world",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="demo_facility_scaled",
                description=(
                    "The <world name> value inside the selected SDF file"
                ),
            ),
            # Mid-point of the south aisle. The equipment island is centred in
            # the hall and leaves a 0.59-0.61 m corridor on all four sides, so
            # the patrol route is a closed loop around the outer wall rather
            # than the dead-end aisle the unscaled layout had.
            DeclareLaunchArgument("spawn_x", default_value="0.0975"),
            DeclareLaunchArgument("spawn_y", default_value="-1.4121"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "simulation_mode",
                default_value="kinematic",
                description=(
                    "Fortress drive mode: stable kinematic integration or "
                    "physical Mecanum wheel-contact validation."
                ),
            ),
            DeclareLaunchArgument("visualize_sensors", default_value="false"),
            DeclareLaunchArgument(
                "include_dispenser",
                default_value="true",
                description="Attach the provisional rear dispenser geometry",
            ),
            DeclareLaunchArgument("dispenser_mass", default_value="1.2"),
            DeclareLaunchArgument(
                "heat_source_profile",
                default_value="",
                description="JSON profile for deterministic synthetic heat sources",
            ),
            DeclareLaunchArgument(
                "use_thermal_pipeline",
                default_value="false",
                description=(
                    "Fuse simulated thermal/depth images into the common "
                    "thermal point-cloud contract. False keeps the "
                    "deterministic profile detector."
                ),
            ),
            DeclareLaunchArgument(
                "thermal_history_path",
                default_value="",
                description="JSONL path written only when record_visit is called",
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                [
                    EnvironmentVariable(
                        "IGN_GAZEBO_RESOURCE_PATH", default_value=""
                    ),
                    os.pathsep,
                    str(simulation_share.parent),
                    # The facility world resolves its equipment through bare
                    # "model://<name>" URIs, so the directory holding those
                    # model folders has to be on the resource path itself.
                    os.pathsep,
                    str(simulation_share / "models"),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "gz_args": ["-r -v 3 ", world],
                    "gz_version": "6",
                    "on_exit_shutdown": "true",
                }.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "gz_args": ["-r -s -v 3 ", world],
                    "gz_version": "6",
                    "on_exit_shutdown": "true",
                }.items(),
                condition=UnlessCondition(gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="hazard_guard_gz_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo",
                    "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo",
                    "/depth_camera/points@sensor_msgs/msg/PointCloud2"
                    "[gz.msgs.PointCloudPacked",
                    "/thermal_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/thermal_camera/camera_info@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo",
                ],
                remappings=[
                    ("/camera", "/camera/image_raw"),
                    ("/depth_camera", "/depth_camera/image_raw"),
                    ("/thermal_camera", "/thermal_camera/image_raw"),
                ],
                parameters=[
                    {
                        (
                            "qos_overrides./clock.publisher.durability"
                        ): "volatile",
                        (
                            "qos_overrides./scan.publisher.reliability"
                        ): "best_effort",
                    }
                ],
                output="screen",
            ),
            Node(
                package="hazard_guard_mock_robot",
                executable="mock_robot",
                name="hazard_guard_simulation_telemetry",
                output="screen",
                parameters=[
                    {
                        "robot_id": "rosmaster-m1-fortress",
                        "publish_rate_hz": 2.0,
                        "initial_battery_percent": 78.0,
                        "use_simulation_inputs": True,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="hazard_guard_mock_robot",
                executable="thermal_detector_mock",
                name="hazard_guard_simulation_thermal_detector",
                output="screen",
                parameters=[
                    {
                        "camera_model": TMC160B.model,
                        "horizontal_fov_deg": TMC160B.horizontal_fov_deg,
                        "range_min_m": 0.0,
                        "range_max_m": TMC160B.visualization_range_m,
                        "sensor_frame": TMC160B.sensor_frame,
                        "publish_rate_hz": 2.0,
                        "heat_source_profile": heat_source_profile,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=UnlessCondition(use_thermal_pipeline),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        thermal_analysis_share
                        / "launch"
                        / "thermal_pipeline.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "simulated": "true",
                    "thermal_image_topic": "/thermal_camera/image_raw",
                    "thermal_info_topic": "/thermal_camera/camera_info",
                    "depth_image_topic": "/depth_camera/image_raw",
                    "depth_info_topic": "/depth_camera/camera_info",
                    "thermal_scale": "0.01",
                    "thermal_offset_c": "-273.15",
                    "history_path": thermal_history_path,
                }.items(),
                condition=IfCondition(use_thermal_pipeline),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            world_name,
                            "-name",
                            "hazard_guard_m1",
                            "-allow_renaming",
                            "false",
                            "-topic",
                            "robot_description",
                            "-x",
                            spawn_x,
                            "-y",
                            spawn_y,
                            "-z",
                            spawn_z,
                            "-Y",
                            spawn_yaw,
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
