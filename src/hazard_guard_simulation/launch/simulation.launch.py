import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
    simulation_share = Path(get_package_share_directory("hazard_guard_simulation"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    default_world = simulation_share / "worlds" / "facility_map.sdf"
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
                default_value="facility_map",
                description="The <world name> value inside the selected SDF file",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.60"),
            DeclareLaunchArgument("spawn_y", default_value="0.70"),
            DeclareLaunchArgument("spawn_z", default_value="0.04"),
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
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                [
                    EnvironmentVariable(
                        "IGN_GAZEBO_RESOURCE_PATH", default_value=""
                    ),
                    os.pathsep,
                    str(simulation_share.parent),
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
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
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
                    "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/depth_camera/points@sensor_msgs/msg/PointCloud2"
                    "[gz.msgs.PointCloudPacked",
                    "/thermal_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                ],
                remappings=[
                    ("/camera", "/camera/image_raw"),
                    ("/depth_camera", "/depth_camera/image_raw"),
                    ("/thermal_camera", "/thermal_camera/image_raw"),
                ],
                parameters=[
                    {
                        "qos_overrides./clock.publisher.durability": "volatile",
                        "qos_overrides./scan.publisher.reliability": "best_effort",
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
                        "horizontal_fov_deg": 56.0,
                        "range_min_m": 0.1,
                        "range_max_m": 5.0,
                        "sensor_frame": "thermal_camera_link",
                        "publish_rate_hz": 2.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
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
