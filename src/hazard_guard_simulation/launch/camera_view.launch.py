"""Open one rqt_image_view window per camera stream, plus person detection.

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
from launch_ros.parameter_descriptions import ParameterValue

# label -> (launch argument, default topic, shown by default)
# The thermal window defaults to the overlay: the raw mono16 is temperature,
# not brightness, so it shows up as a near-black frame, and the marker on top
# is what makes the calibration checkable at a glance.
STREAMS = (
    ("thermal", "thermal_topic", "/thermal_camera/image_overlay", "true"),
    ("depth", "depth_topic", "/depth_camera/image_raw", "true"),
    ("rgb", "rgb_topic", "/camera/image_raw", "false"),
    # The person stream is the RGB frame with detection boxes drawn on it, so
    # the raw RGB window stays off by default - the same picture, twice.
    ("person", "person_topic", "/person_detector/image", "true"),
)


def generate_launch_description() -> LaunchDescription:
    actions = [
        DeclareLaunchArgument(
            "min_temp_c",
            default_value="10.0",
            description="Blue end of the thermal colour map",
        ),
        DeclareLaunchArgument(
            "max_temp_c",
            default_value="60.0",
            description="Red end of the thermal colour map",
        ),
        # thermal_camera_info.py is not started here: simulation.launch.py owns
        # it now, because the thermal map needs those intrinsics whether or not
        # anyone has a viewer open.
        Node(
            package="hazard_guard_simulation",
            executable="thermal_overlay.py",
            name="thermal_overlay",
            output="screen",
        ),
        DeclareLaunchArgument(
            "person_model",
            default_value="yolo11n.pt",
            description="YOLO weights; downloaded next to the working "
            "directory on first use if the file is not there",
        ),
        DeclareLaunchArgument(
            "person_conf",
            default_value="0.25",
            description="Detection threshold; the workers are scaled to the "
            "miniature facility, so they are small in frame",
        ),
        DeclareLaunchArgument("person_imgsz", default_value="960"),
        Node(
            package="hazard_guard_simulation",
            executable="person_detector.py",
            name="person_detector",
            output="screen",
            parameters=[
                {
                    # Launch arguments arrive as strings; the node declares
                    # these as double and int, and a string would be rejected.
                    "model": ParameterValue(
                        LaunchConfiguration("person_model"), value_type=str
                    ),
                    "conf": ParameterValue(
                        LaunchConfiguration("person_conf"), value_type=float
                    ),
                    "imgsz": ParameterValue(
                        LaunchConfiguration("person_imgsz"), value_type=int
                    ),
                }
            ],
        ),
        Node(
            package="hazard_guard_simulation",
            executable="thermal_colorize.py",
            name="thermal_colorize",
            output="screen",
            parameters=[
                {
                    "min_temp_c": LaunchConfiguration("min_temp_c"),
                    "max_temp_c": LaunchConfiguration("max_temp_c"),
                }
            ],
        ),
    ]
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
