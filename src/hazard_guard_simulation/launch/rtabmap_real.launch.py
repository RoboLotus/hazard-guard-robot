"""Live RGB-D RTAB-Map for the physical ROSMASTER M1 / HP60C setup.

SLAM Toolbox remains the authority for the 2D ``map -> odom`` transform.
RTAB-Map publishes its independent ``rtabmap_map -> odom`` transform so its
3D reconstruction can be viewed without affecting Nav2's 2D map.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    database_path = LaunchConfiguration("database_path")
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
                arguments=["0", "0", "0", "1.570796", "3.141592", "1.570796", "camera_Link", "ascamera_hp60c_camera_link_0"],
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
                        "odom_frame_id": "odom",
                        "map_frame_id": "rtabmap_map",
                        "database_path": database_path,
                        "publish_tf": True,
                        "subscribe_rgbd": True,
                        "subscribe_scan": True,
                        "approx_sync": True,
                        "qos_image": 2,
                        "qos_camera_info": 2,
                        "qos_scan": 2,
                        "qos_odom": 2,
                        "Reg/Strategy": "1",
                        "Reg/Force3DoF": "true",
                        "RGBD/NeighborLinkRefining": "true",
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
                remappings=[
                    ("input", "/hazard_guard/rtabmap/cloud_frame_limited"),
                    ("output", "/hazard_guard/rtabmap/cloud_frame"),
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
                        # Assemble in the continuously available physical odom
                        # tree. RTAB-Map's map transform can be published after
                        # the first optimized graph update, which otherwise
                        # leaves the camera tree temporarily disconnected.
                        "fixed_frame_id": "odom",
                        "frame_id": "odom",
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
        ]
    )
