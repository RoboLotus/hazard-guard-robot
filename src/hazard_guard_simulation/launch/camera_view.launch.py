"""Open one rqt_image_view window per camera stream.

For the thermal-to-depth calibration work: the two cameras sit on different
links (thermal at base_link +0.158/0/+0.112, depth at +0.0859/0/+0.0941, so a
72 x 18 mm baseline) and the streams have to be looked at side by side before
any extrinsics are fitted.

A simulation has to be running already - this launch only opens viewers:

    ros2 launch hazard_guard_simulation simulation.launch.py gui:=true
    ros2 launch hazard_guard_simulation camera_view.launch.py

The thermal stream is mono16 carrying temperature, not brightness, so the
window looks flat until "Dynamic range" is ticked in its toolbar. Values are
Kelvin x 100 (29315 = 20.0 C).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# label -> (launch argument, default topic, shown by default)
STREAMS = (
    ("thermal", "thermal_topic", "/thermal_camera/image_raw", "true"),
    ("depth", "depth_topic", "/depth_camera/image_raw", "true"),
    ("rgb", "rgb_topic", "/camera/image_raw", "false"),
)


def generate_launch_description() -> LaunchDescription:
    actions = []
    for label, argument, topic, shown in STREAMS:
        actions.append(DeclareLaunchArgument(argument, default_value=topic))
        actions.append(
            DeclareLaunchArgument(
                f"show_{label}",
                default_value=shown,
                description=f"Open a viewer for the {label} stream",
            )
        )
        actions.append(
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                # rqt names its own node, so two viewers do not collide.
                arguments=[LaunchConfiguration(argument)],
                condition=IfCondition(LaunchConfiguration(f"show_{label}")),
                output="screen",
            )
        )
    return LaunchDescription(actions)
