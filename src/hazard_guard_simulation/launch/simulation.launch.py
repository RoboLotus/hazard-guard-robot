import os
from pathlib import Path

import yaml
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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


THERMAL_OPTICAL_ARGUMENTS = (
    "thermal_optical_x", "thermal_optical_y", "thermal_optical_z",
    "thermal_optical_roll", "thermal_optical_pitch", "thermal_optical_yaw",
)


def thermal_extrinsic_arguments(simulation_share: Path) -> list:
    """Extra xacro arguments carrying the applied thermal calibration.

    Empty when nothing has been applied, which leaves the mounting-drawing
    defaults in the xacro untouched. tools/apply_calibration.py writes the
    file; HAZARD_GUARD_THERMAL_EXTRINSIC points somewhere else for the
    perturbation runs that check a calibration recovers a known offset.
    """
    path = Path(os.environ.get(
        "HAZARD_GUARD_THERMAL_EXTRINSIC",
        simulation_share / "config" / "thermal_extrinsic.yaml"))
    if not path.is_file():
        return []
    values = yaml.safe_load(path.read_text()) or {}
    missing = [name for name in THERMAL_OPTICAL_ARGUMENTS if name not in values]
    # A half-written file would silently mix calibrated and drawing values,
    # and the mixture is wrong in a way no picture makes obvious.
    if missing:
        raise RuntimeError(f"{path} 에 인자가 빠졌습니다: {', '.join(missing)}")
    print(f"[simulation.launch] 열화상 외부파라미터 적용: {path}")
    return [f" {name}:={values[name]}" for name in THERMAL_OPTICAL_ARGUMENTS]


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
    cmd_vel_ros_topic = LaunchConfiguration("cmd_vel_ros_topic")
    use_thermal_pipeline = LaunchConfiguration("use_thermal_pipeline")
    thermal_history_path = LaunchConfiguration("thermal_history_path")
    thermal_baseline_path = LaunchConfiguration("thermal_baseline_path")
    thermal_baseline_collection_path = LaunchConfiguration(
        "thermal_baseline_collection_path"
    )
    thermal_baseline_minimum_valid_visits = LaunchConfiguration(
        "thermal_baseline_minimum_valid_visits"
    )

    thermal_extrinsic = thermal_extrinsic_arguments(simulation_share)

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
                *thermal_extrinsic,
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
                "cmd_vel_ros_topic",
                default_value="/cmd_vel",
                description=(
                    "ROS-side velocity topic bridged to Gazebo /cmd_vel. "
                    "Use /cmd_vel_safe when the person-safety gate is enabled."
                ),
            ),
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
                default_value="~/.local/share/hazard_guard/simulation_thermal_history.jsonl",
                description="Persistent simulation patrol history JSONL path",
            ),
            DeclareLaunchArgument(
                "thermal_baseline_path",
                default_value="~/.local/share/hazard_guard/simulation_thermal_baselines.json",
            ),
            DeclareLaunchArgument(
                "thermal_baseline_collection_path",
                default_value=(
                    "~/.local/share/hazard_guard/"
                    "simulation_thermal_baseline_collection.json"
                ),
            ),
            DeclareLaunchArgument(
                "thermal_baseline_minimum_valid_visits",
                default_value="10",
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
                    ("/cmd_vel", cmd_vel_ros_topic),
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
                # The intrinsics the bridge above deliberately does not carry.
                # Anything geometric on the thermal stream needs these.
                package="hazard_guard_simulation",
                executable="thermal_camera_info.py",
                name="thermal_camera_info",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
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
                    "baseline_path": thermal_baseline_path,
                    "baseline_collection_path": (
                        thermal_baseline_collection_path
                    ),
                    "baseline_minimum_valid_visits": (
                        thermal_baseline_minimum_valid_visits
                    ),
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
