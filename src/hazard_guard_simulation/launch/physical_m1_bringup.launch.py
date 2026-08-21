"""ROSMASTER M1 bringup with a motor-only command topic boundary.

The vendor launch starts both the motor subscriber and joystick publisher in
one include. A group-wide remap would therefore let the joystick bypass the
safety gate. This wrapper mirrors the vendor M1 bringup but applies the
configurable command topic only to ``Mcnamu_driver_M1``.
"""

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
    motor_cmd_vel_topic = LaunchConfiguration("motor_cmd_vel_topic")
    pub_odom_tf = LaunchConfiguration("pub_odom_tf")
    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
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
            DeclareLaunchArgument("pub_odom_tf", default_value="false"),
            DeclareLaunchArgument("motor_cmd_vel_topic", default_value="/cmd_vel"),
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
            # This is the only node whose input is redirected to the final
            # safety-gated topic. Joystick and Nav2 keep publishing /cmd_vel.
            Node(
                package="yahboomcar_bringup",
                executable="Mcnamu_driver_M1",
                name="driver_node",
                remappings=[("cmd_vel", motor_cmd_vel_topic)],
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
            Node(package="yahboomcar_ctrl", executable="yahboom_joy_M1"),
            Node(package="joy", executable="joy_node"),
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
