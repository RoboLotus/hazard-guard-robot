"""ROSMASTER M1 bringup with a motor-only command topic boundary.

The vendor launch starts both the motor subscriber and joystick publisher in
one include. A group-wide remap would therefore let the joystick bypass the
safety gate. This wrapper mirrors the vendor M1 bringup but applies the
configurable command topic only to ``Mcnamu_driver_M1``.
"""

import subprocess
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_path("yahboomcar_description")
    bringup_share = Path(get_package_share_directory("yahboomcar_bringup"))
    localization_share = Path(get_package_share_directory("robot_localization"))
    lidar_share = Path(get_package_share_directory("ydlidar_ros2_driver"))

    model = LaunchConfiguration("model")
    gui = LaunchConfiguration("gui")
    start_joystick = LaunchConfiguration("start_joystick")
    motor_cmd_vel_topic = LaunchConfiguration("motor_cmd_vel_topic")
    pub_odom_tf = LaunchConfiguration("pub_odom_tf")
    battery_voltage_topic = LaunchConfiguration("battery_voltage_topic")
    battery_state_topic = LaunchConfiguration("battery_state_topic")
    battery_window_size = LaunchConfiguration("battery_window_size")
    battery_empty_voltage_v = LaunchConfiguration("battery_empty_voltage_v")
    battery_full_voltage_v = LaunchConfiguration("battery_full_voltage_v")
    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    # hazardguard-base.service (hazard_guard_iot) keeps Mcnamu_driver_M1 and
    # battery_telemetry running while the robot is idle so the SmartThings app
    # shows battery. /dev/myserial can only be opened by one process, so stop
    # that service before starting our own driver and bring it back when this
    # launch shuts down. /etc/sudoers.d/hazardguard (installed with the
    # service) allows exactly these two commands without a password. If the
    # service is not installed, ``sudo -n`` fails fast and the driver still
    # starts, so the launch behaves as before.
    release_serial_service = ExecuteProcess(
        cmd=["sudo", "-n", "systemctl", "stop", "hazardguard-base.service"],
        name="release_serial_service",
        output="log",
    )

    def restore_serial_service(event, context):
        # Runs synchronously: launch does not start new processes while it
        # is shutting down, so an ExecuteProcess here would never run.
        subprocess.run(
            ["sudo", "-n", "systemctl", "start", "hazardguard-base.service"],
            check=False,
        )

    # This is the only node whose input is redirected to the final
    # safety-gated topic. Joystick and Nav2 keep publishing /cmd_vel.
    motor_driver = Node(
        package="yahboomcar_bringup",
        executable="Mcnamu_driver_M1",
        name="driver_node",
        remappings=[("cmd_vel", motor_cmd_vel_topic)],
    )
    battery_telemetry = Node(
        package="hazard_guard_robot_telemetry",
        executable="battery_telemetry",
        name="hazard_guard_battery_telemetry",
        output="screen",
        parameters=[
            {
                "input_topic": battery_voltage_topic,
                "output_topic": battery_state_topic,
                "window_size": ParameterValue(
                    battery_window_size, value_type=int
                ),
                "empty_voltage_v": ParameterValue(
                    battery_empty_voltage_v, value_type=float
                ),
                "full_voltage_v": ParameterValue(
                    battery_full_voltage_v, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=str(
                    description_share / "urdf" / "yahboomcar_M1.urdf.xacro"
                ),
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("start_joystick", default_value="true"),
            DeclareLaunchArgument("pub_odom_tf", default_value="false"),
            DeclareLaunchArgument("motor_cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "battery_voltage_topic", default_value="/voltage"
            ),
            DeclareLaunchArgument(
                "battery_state_topic", default_value="/hazard_guard/battery"
            ),
            DeclareLaunchArgument("battery_window_size", default_value="20"),
            DeclareLaunchArgument(
                "battery_empty_voltage_v", default_value="10.5"
            ),
            DeclareLaunchArgument(
                "battery_full_voltage_v", default_value="12.6"
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            release_serial_service,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=release_serial_service,
                    on_exit=[motor_driver, battery_telemetry],
                )
            ),
            RegisterEventHandler(
                OnShutdown(on_shutdown=restore_serial_service)
            ),
            Node(
                package="yahboomcar_base_node",
                executable="base_node_M1",
                parameters=[
                    {
                        "pub_odom_tf": pub_odom_tf,
                        "linear_scale_x": 1.0,
                        "linear_scale_y": 1.0,
                    }
                ],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                parameters=[str(bringup_share / "param" / "imu_filter_param.yaml")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(localization_share / "launch" / "ekf_M1_launch.py")
                )
            ),
            Node(
                package="yahboomcar_ctrl",
                executable="yahboom_joy_M1",
                condition=IfCondition(start_joystick),
            ),
            Node(
                package="joy",
                executable="joy_node",
                condition=IfCondition(start_joystick),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(lidar_share / "launch" / "ydlidar_launch.py")
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser",
                arguments=[
                    "0.0060585",
                    "0",
                    "0.14912",
                    "0",
                    "3.1415",
                    "3.1415",
                    "base_link",
                    "laser",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_to_hp60c",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "1.570796",
                    "3.141592",
                    "1.570796",
                    "camera_Link",
                    "ascamera_hp60c_camera_link_0",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="hp60c_link_to_color",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "ascamera_hp60c_camera_link_0",
                    "ascamera_hp60c_color_0",
                ],
            ),
        ]
    )
