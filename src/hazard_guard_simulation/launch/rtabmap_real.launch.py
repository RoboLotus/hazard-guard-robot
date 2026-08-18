"""Live RGB-D RTAB-Map for the physical ROSMASTER M1 / HP60C setup.

SLAM Toolbox remains the authority for the 2D ``map -> odom`` transform.
RTAB-Map keeps its optimized ``rtabmap_map`` coordinates in map messages but
does not publish another parent transform for ``odom``. This preserves the
single TF tree required by Nav2 while still allowing 3D map comparison.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_cloud_configuration(
    fixed_frame: str,
    output_frame: str,
    stamp_mode: str,
) -> None:
    """Reject frame/timestamp combinations known to distort physical data."""

    frames = {fixed_frame, output_frame}
    if not frames <= {"odom", "map"}:
        raise ValueError("cloud frames must be 'odom' or 'map'")
    if "map" in frames and stamp_mode == "latest":
        raise ValueError(
            "cloud_stamp_mode=latest cannot be combined with a map-frame "
            "assembler; measure the sensor skew and use preserve or offset"
        )


def validate_launch_configuration(context):
    validate_cloud_configuration(
        LaunchConfiguration("cloud_fixed_frame").perform(context),
        LaunchConfiguration("cloud_output_frame").perform(context),
        LaunchConfiguration("cloud_stamp_mode").perform(context),
    )
    return []


def generate_launch_description() -> LaunchDescription:
    database_path = LaunchConfiguration("database_path")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    storage_path = LaunchConfiguration("storage_path")
    cloud_decimation = LaunchConfiguration("cloud_decimation")
    cloud_voxel_size = LaunchConfiguration("cloud_voxel_size")
    cloud_linear_update = LaunchConfiguration("cloud_linear_update")
    cloud_angular_update = LaunchConfiguration("cloud_angular_update")

    common_camera_remaps = [
        ("rgb/image", "/ascamera_hp60c/camera_publisher/rgb0/image"),
        ("rgb/camera_info", "/ascamera_hp60c/camera_publisher/rgb0/camera_info"),
        ("depth/image", "/ascamera_hp60c/camera_publisher/depth0/image_raw"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=str(
                    Path.home()
                    / "RoboLotus/hazard-guard-robot/runtime/maps/real_rtabmap_live.db"
                ),
            ),
            DeclareLaunchArgument(
                "storage_path",
                default_value=str(
                    Path.home() / "RoboLotus/hazard-guard-robot/runtime/maps"
                ),
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value="odom",
                choices=["odom", "map"],
                description=(
                    "External pose frame stored in RTAB-Map. Use map only "
                    "after AMCL localization is active."
                ),
            ),
            DeclareLaunchArgument(
                "cloud_normal_points", default_value="9000"
            ),
            DeclareLaunchArgument(
                "cloud_high_load_points", default_value="4500"
            ),
            DeclareLaunchArgument(
                "cloud_normal_input_hz", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "cloud_high_load_input_hz", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "cloud_normal_surface_hz", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "cloud_high_load_surface_hz", default_value="0.5"
            ),
            DeclareLaunchArgument("cloud_decimation", default_value="2"),
            DeclareLaunchArgument("cloud_voxel_size", default_value="0.03"),
            DeclareLaunchArgument("cloud_linear_update", default_value="0.10"),
            DeclareLaunchArgument(
                "cloud_angular_update",
                default_value="0.10472",
            ),
            DeclareLaunchArgument(
                "cloud_stamp_mode",
                default_value="latest",
                choices=["preserve", "offset", "latest"],
                description="preserve, offset, or latest (zero stamp)",
            ),
            DeclareLaunchArgument(
                "cloud_stamp_offset_sec",
                default_value="0.0",
                description=(
                    "Seconds added to the cloud stamp in offset mode; "
                    "use a negative value when the camera clock is ahead"
                ),
            ),
            DeclareLaunchArgument(
                "sync_diagnostics",
                default_value="false",
                description=(
                    "Enable short-lived physical sensor timing diagnostics"
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_registration_strategy",
                default_value="1",
                choices=["0", "1", "2"],
                description="RTAB-Map registration: 0=Visual, 1=ICP, 2=Visual+ICP",
            ),
            DeclareLaunchArgument(
                "subscribe_scan",
                default_value="true",
                description=(
                    "Use LiDAR constraints inside RTAB-Map. The second-pass "
                    "RGB-D workflow disables this because localization owns pose."
                ),
            ),
            DeclareLaunchArgument(
                "neighbor_link_refining",
                default_value="true",
                description="Refine neighboring graph links with registration.",
            ),
            DeclareLaunchArgument(
                "proximity_by_space",
                default_value="true",
                description="Search nearby graph nodes for loop constraints.",
            ),
            DeclareLaunchArgument(
                "loop_closure_threshold",
                default_value="0.11",
                description=(
                    "RTAB-Map loop closure threshold. Use 0.0 when an external "
                    "localization stack is the only pose authority."
                ),
            ),
            DeclareLaunchArgument(
                "cloud_fixed_frame",
                default_value="odom",
                choices=["odom", "map"],
                description="Frame used while accumulating visualization clouds",
            ),
            DeclareLaunchArgument(
                "cloud_output_frame",
                default_value="odom",
                choices=["odom", "map"],
                description="Frame written into the assembled visualization cloud",
            ),
            DeclareLaunchArgument(
                "optimized_cloud",
                default_value="false",
                description=(
                    "Publish RTAB-Map graph-optimized comparison cloud on an "
                    "internal topic; disabled by default to protect Jetson load"
                ),
            ),
            OpaqueFunction(function=validate_launch_configuration),
            # The camera publishes its internal TF tree only. These transforms
            # attach it to the physical M1 frame tree used by SLAM Toolbox.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_mount",
                arguments=["0.085871", "0", "0.094136", "0", "0", "0", "base_link", "camera_Link"],
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
                package="rtabmap_sync",
                executable="rgbd_sync",
                namespace="rtabmap",
                name="rgbd_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.08,
                        "sync_queue_size": 30,
                        "qos": 2,
                    }
                ],
                remappings=common_camera_remaps,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                namespace="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "frame_id": "base_link",
                        "odom_frame_id": odom_frame_id,
                        "map_frame_id": "rtabmap_map",
                        "database_path": database_path,
                        # SLAM Toolbox exclusively owns map -> odom for Nav2.
                        # Publishing rtabmap_map -> odom would give odom two
                        # parents and corrupt the physical robot TF tree.
                        "publish_tf": False,
                        "subscribe_rgbd": True,
                        "subscribe_scan": ParameterValue(
                            LaunchConfiguration("subscribe_scan"),
                            value_type=bool,
                        ),
                        "approx_sync": True,
                        "qos_image": 2,
                        "qos_camera_info": 2,
                        "qos_scan": 2,
                        "qos_odom": 2,
                        "Reg/Strategy": ParameterValue(
                            LaunchConfiguration("rtabmap_registration_strategy"),
                            # RTAB-Map exposes core parameters as strings even
                            # when their documented values are numeric.
                            value_type=str,
                        ),
                        "Reg/Force3DoF": "true",
                        "RGBD/NeighborLinkRefining": ParameterValue(
                            LaunchConfiguration("neighbor_link_refining"),
                            value_type=str,
                        ),
                        "RGBD/ProximityBySpace": ParameterValue(
                            LaunchConfiguration("proximity_by_space"),
                            value_type=str,
                        ),
                        "Rtabmap/LoopThr": ParameterValue(
                            LaunchConfiguration("loop_closure_threshold"),
                            value_type=str,
                        ),
                        "RGBD/OptimizeMaxError": "10.0",
                        "Vis/MinInliers": "20",
                        "Grid/FromDepth": "true",
                        "Grid/3D": "true",
                        "Grid/RangeMin": "0.2",
                        "Grid/RangeMax": "4.0",
                        "Mem/IncrementalMemory": "true",
                        "Rtabmap/DetectionRate": "2.0",
                    }
                ],
                remappings=[
                    ("rgbd_image", "rgbd_image"),
                    ("scan", "/scan"),
                    ("odom", "/odom"),
                ],
                # Keep the database supplied by the mapping session.  "-d"
                # would delete it every time the physical stack is started.
                arguments=[],
            ),
            Node(
                package="hazard_guard_simulation",
                executable="camera_info_relay.py",
                name="rgb_camera_info_relay",
                output="screen",
                remappings=[
                    ("input", "/ascamera_hp60c/camera_publisher/rgb0/camera_info"),
                    ("output", "/hazard_guard/camera/rgb/camera_info"),
                ],
            ),
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyzrgb",
                namespace="rtabmap",
                name="color_cloud_frame",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.08,
                        "sync_queue_size": 20,
                        "topic_queue_size": 5,
                        "qos": 2,
                        "qos_camera_info": 2,
                        "decimation": ParameterValue(
                            cloud_decimation,
                            value_type=int,
                        ),
                        "min_depth": 0.2,
                        "max_depth": 4.0,
                        "filter_nans": True,
                    }
                ],
                remappings=[
                    ("rgb/image", "/ascamera_hp60c/camera_publisher/rgb0/image"),
                    ("rgb/camera_info", "/hazard_guard/camera/rgb/camera_info"),
                    ("depth/image", "/ascamera_hp60c/camera_publisher/depth0/image_raw"),
                    ("cloud", "/hazard_guard/rtabmap/cloud_frame_generated"),
                ],
            ),
            # This guard affects only the 3D visualization path. It bounds the
            # point count/rate and pauses map integration under sustained
            # Jetson load without touching RTAB-Map odometry, SLAM Toolbox or
            # Nav2. The public cumulative-map topic remains unchanged.
            Node(
                package="hazard_guard_simulation",
                executable="adaptive_cloud_guard.py",
                name="adaptive_cloud_guard",
                output="screen",
                parameters=[
                    {
                        "normal_points": ParameterValue(
                            LaunchConfiguration("cloud_normal_points"),
                            value_type=int,
                        ),
                        "high_load_points": ParameterValue(
                            LaunchConfiguration("cloud_high_load_points"),
                            value_type=int,
                        ),
                        "normal_input_hz": ParameterValue(
                            LaunchConfiguration("cloud_normal_input_hz"),
                            value_type=float,
                        ),
                        "high_load_input_hz": ParameterValue(
                            LaunchConfiguration("cloud_high_load_input_hz"),
                            value_type=float,
                        ),
                        "normal_surface_hz": ParameterValue(
                            LaunchConfiguration("cloud_normal_surface_hz"),
                            value_type=float,
                        ),
                        "high_load_surface_hz": ParameterValue(
                            LaunchConfiguration("cloud_high_load_surface_hz"),
                            value_type=float,
                        ),
                        "storage_path": storage_path,
                    }
                ],
                remappings=[
                    ("input", "/hazard_guard/rtabmap/cloud_frame_generated"),
                    ("output", "/hazard_guard/rtabmap/cloud_frame_limited"),
                    (
                        "surface_input",
                        "/hazard_guard/rtabmap/cloud_surface_internal",
                    ),
                    ("surface_output", "/hazard_guard/rtabmap/cloud_surface"),
                    # The managed WebUI currently overrides its configured
                    # source with this legacy topic name. Publish the same
                    # cumulative map here so no WebUI change is required.
                    (
                        "surface_compat_output",
                        "/hazard_guard/rtabmap/cloud_frame_raw",
                    ),
                    ("status", "/hazard_guard/rtabmap/cloud_guard/status"),
                ],
            ),
            # HP60C image stamps lead the physical odom TF by several seconds.
            # Re-stamp only the visualization cloud so the assembler can use
            # the latest available odometry transform instead of dropping it.
            Node(
                package="hazard_guard_simulation",
                executable="cloud_stamp_relay.py",
                name="color_cloud_stamp_relay",
                output="screen",
                parameters=[
                    {
                        "stamp_mode": LaunchConfiguration("cloud_stamp_mode"),
                        "stamp_offset_sec": ParameterValue(
                            LaunchConfiguration("cloud_stamp_offset_sec"),
                            value_type=float,
                        ),
                    }
                ],
                remappings=[
                    ("input", "/hazard_guard/rtabmap/cloud_frame_limited"),
                    ("output", "/hazard_guard/rtabmap/cloud_frame"),
                ],
            ),
            Node(
                package="hazard_guard_simulation",
                executable="timestamp_diagnostics.py",
                name="timestamp_diagnostics",
                output="screen",
                condition=IfCondition(LaunchConfiguration("sync_diagnostics")),
                parameters=[
                    {
                        "target_frame": "odom",
                        "report_dir": PathJoinSubstitution(
                            [storage_path, "diagnostics"]
                        ),
                    }
                ],
                remappings=[
                    (
                        "rgb",
                        "/ascamera_hp60c/camera_publisher/rgb0/image",
                    ),
                    (
                        "depth",
                        "/ascamera_hp60c/camera_publisher/depth0/image_raw",
                    ),
                    (
                        "camera_info",
                        "/ascamera_hp60c/camera_publisher/rgb0/camera_info",
                    ),
                    ("odom", "/odom"),
                    ("scan", "/scan"),
                    ("cloud", "/hazard_guard/rtabmap/cloud_frame_generated"),
                ],
            ),
            Node(
                package="rtabmap_util",
                executable="point_cloud_assembler",
                namespace="rtabmap",
                name="color_cloud_assembler",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        # The physical odom tree is the compatible default.
                        # A SLAM Toolbox map-frame run is available only with a
                        # timestamp-aware preserve/offset policy.
                        "fixed_frame_id": LaunchConfiguration(
                            "cloud_fixed_frame"
                        ),
                        "frame_id": LaunchConfiguration("cloud_output_frame"),
                        # Publish the whole current mapping session upstream.
                        # The WebUI replaces each received cloud, so a rolling
                        # buffer here made explored areas disappear visually.
                        "max_clouds": 0,
                        # circular_buffer publishes from the first accepted
                        # cloud. The one-hour window bounds memory while
                        # movement thresholds keep stationary frames out.
                        "assembling_time": 3600.0,
                        "circular_buffer": True,
                        "linear_update": ParameterValue(
                            cloud_linear_update,
                            value_type=float,
                        ),
                        "angular_update": ParameterValue(
                            cloud_angular_update,
                            value_type=float,
                        ),
                        "voxel_size": ParameterValue(
                            cloud_voxel_size,
                            value_type=float,
                        ),
                        "range_min": 0.2,
                        "range_max": 4.0,
                        "wait_for_transform": 0.5,
                        "qos": 2,
                    }
                ],
                remappings=[
                    ("cloud", "/hazard_guard/rtabmap/cloud_frame"),
                    (
                        "assembled_cloud",
                        "/hazard_guard/rtabmap/cloud_surface_internal",
                    ),
                ],
            ),
            # This optional comparison backend rebuilds the cloud from
            # RTAB-Map's optimized graph and node data. It does not replace the
            # public WebUI topic until physical comparison selects a winner.
            Node(
                package="rtabmap_util",
                executable="map_assembler",
                namespace="rtabmap",
                name="optimized_map_assembler",
                output="screen",
                condition=IfCondition(LaunchConfiguration("optimized_cloud")),
                parameters=[
                    {
                        "use_sim_time": False,
                        "map_always_update": True,
                        "map_cleanup": True,
                        "cloud_output_voxelized": True,
                        # Core Grid/* parameters are declared as strings by
                        # map_assembler; map output controls above are bools.
                        "Grid/3D": "true",
                        "Grid/RangeMin": "0.2",
                        "Grid/RangeMax": "4.0",
                        "Grid/CellSize": "0.08",
                    }
                ],
                remappings=[
                    (
                        "cloud_map",
                        "/hazard_guard/rtabmap/cloud_surface_optimized",
                    )
                ],
            ),
        ]
    )
