"""Launch the HazardGuard person detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("hazard_guard_person_detection"), "config", "default.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("model_path", default_value="yolo11n.pt"),
            DeclareLaunchArgument("device", default_value=""),
            DeclareLaunchArgument("simulated", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "depth_registration_verified",
                default_value="false",
            ),
            Node(
                package="hazard_guard_person_detection",
                executable="person_detection_node",
                name="person_detection",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "device": LaunchConfiguration("device"),
                        "simulated": ParameterValue(
                            LaunchConfiguration("simulated"), value_type=bool
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "depth_registration_verified": ParameterValue(
                            LaunchConfiguration("depth_registration_verified"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
