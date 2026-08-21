"""Launch the physical ThermoEye camera and an optional color viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_gui", default_value="false"),
            DeclareLaunchArgument("calibration_file", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="thermal_camera_optical_frame"),
            DeclareLaunchArgument("min_temp_c", default_value="10.0"),
            DeclareLaunchArgument("max_temp_c", default_value="60.0"),
            DeclareLaunchArgument("color_scale_mode", default_value="sdk"),
            DeclareLaunchArgument("relative_low_percentile", default_value="2.0"),
            DeclareLaunchArgument("relative_high_percentile", default_value="98.0"),
            DeclareLaunchArgument("sdk_noise_filtering", default_value="true"),
            Node(
                package="hazard_guard_simulation",
                executable="thermal_camera_publisher.py",
                name="thermal_camera_publisher",
                output="screen",
                parameters=[
                    {
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "min_temp_c": ParameterValue(
                            LaunchConfiguration("min_temp_c"), value_type=float
                        ),
                        "max_temp_c": ParameterValue(
                            LaunchConfiguration("max_temp_c"), value_type=float
                        ),
                        "color_scale_mode": LaunchConfiguration("color_scale_mode"),
                        "relative_low_percentile": ParameterValue(
                            LaunchConfiguration("relative_low_percentile"),
                            value_type=float,
                        ),
                        "relative_high_percentile": ParameterValue(
                            LaunchConfiguration("relative_high_percentile"),
                            value_type=float,
                        ),
                        "sdk_noise_filtering": ParameterValue(
                            LaunchConfiguration("sdk_noise_filtering"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="thermal_camera_view",
                arguments=["--clear-config", "/thermal_camera/image_color"],
                condition=IfCondition(LaunchConfiguration("show_gui")),
                output="screen",
            ),
        ]
    )
