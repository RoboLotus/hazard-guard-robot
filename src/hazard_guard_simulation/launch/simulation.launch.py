import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hazard_guard_sensor_config import TMC160B
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
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
            # Always a headless server, with the GUI as its own process when
            # asked for. Letting `ign gazebo <world>` start both forks the
            # server out of an already-threaded process, and that fork hangs:
            # the server never advertises a topic, the GUI polls "requesting
            # list of world names" forever, and the robot spawn below dies with
            # "Request to create entity ... timed out". Reproduced on every
            # gui:=true run; headless runs were never affected.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "gz_args": ["-r -s -v 3 ", world],
                    "gz_version": "6",
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            # The GUI retries the world list on its own, so it can start
            # straight away and simply wait for the server.
            ExecuteProcess(
                cmd=["ign", "gazebo", "-g", "--force-version", "6"],
                output="screen",
                condition=IfCondition(gui),
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
                    # Image topics are one level deep so that Fortress derives
                    # a per-camera /<name>/camera_info; the remappings below
                    # put the ROS names back to <name>/image_raw.
                    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo",
                    "/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo"
                    "[gz.msgs.CameraInfo",
                    "/depth_camera/image/points@sensor_msgs/msg/PointCloud2"
                    "[gz.msgs.PointCloudPacked",
                    "/thermal_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    # No thermal camera_info here on purpose. Fortress fills it
                    # from the default camera (fx 277, centre 160x120 for a
                    # 160x120 / 57 deg sensor), so thermal_camera_info.py
                    # publishes the real intrinsics instead.
                ],
                remappings=[
                    ("/camera/image", "/camera/image_raw"),
                    ("/depth_camera/image", "/depth_camera/image_raw"),
                    ("/depth_camera/image/points", "/depth_camera/points"),
                    ("/thermal_camera/image", "/thermal_camera/image_raw"),
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
