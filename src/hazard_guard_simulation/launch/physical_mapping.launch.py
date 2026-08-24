"""WebUI-managed physical mapping for 2D or 2D + RGB-D sessions."""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(
    package: str,
    filename: str,
    arguments: Mapping[str, Any] | None = None,
    *,
    condition: Any | None = None,
) -> IncludeLaunchDescription:
    package_share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    workspace = Path(
        os.getenv("HAZARD_GUARD_WORKSPACE", os.getcwd())
    ).expanduser().resolve()
    database_path = LaunchConfiguration("database_path")
    storage_path = LaunchConfiguration("storage_path")
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    enable_thermal_mapping = LaunchConfiguration("enable_thermal_mapping")
    start_thermal_camera = LaunchConfiguration("start_thermal_camera")
    thermal_mapping_condition = IfCondition(
        PythonExpression(
            [
                "'true' if '", enable_rtabmap,
                "'.lower() == 'true' and '", enable_thermal_mapping,
                "'.lower() == 'true' else 'false'",
            ]
        )
    )
    thermal_camera_condition = IfCondition(
        PythonExpression(
            [
                "'true' if '", enable_rtabmap,
                "'.lower() == 'true' and '", enable_thermal_mapping,
                "'.lower() == 'true' and '", start_thermal_camera,
                "'.lower() == 'true' else 'false'",
            ]
        )
    )
    cloud_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "cloud_normal_points",
            "cloud_high_load_points",
            "cloud_normal_input_hz",
            "cloud_high_load_input_hz",
            "cloud_normal_surface_hz",
            "cloud_high_load_surface_hz",
            "cloud_decimation",
            "cloud_voxel_size",
            "cloud_linear_update",
            "cloud_angular_update",
        )
    }
    cloud_defaults = {
        "cloud_normal_points": os.getenv(
            "HAZARD_GUARD_CLOUD_NORMAL_POINTS", "9000"
        ),
        "cloud_high_load_points": os.getenv(
            "HAZARD_GUARD_CLOUD_HIGH_LOAD_POINTS", "4500"
        ),
        "cloud_normal_input_hz": os.getenv(
            "HAZARD_GUARD_CLOUD_NORMAL_INPUT_HZ", "8.0"
        ),
        "cloud_high_load_input_hz": os.getenv(
            "HAZARD_GUARD_CLOUD_HIGH_LOAD_INPUT_HZ", "4.0"
        ),
        "cloud_normal_surface_hz": os.getenv(
            "HAZARD_GUARD_CLOUD_NORMAL_SURFACE_HZ", "1.0"
        ),
        "cloud_high_load_surface_hz": os.getenv(
            "HAZARD_GUARD_CLOUD_HIGH_LOAD_SURFACE_HZ", "0.5"
        ),
        "cloud_decimation": os.getenv(
            "HAZARD_GUARD_CLOUD_DECIMATION", "2"
        ),
        "cloud_voxel_size": os.getenv(
            "HAZARD_GUARD_CLOUD_VOXEL_SIZE", "0.03"
        ),
        "cloud_linear_update": os.getenv(
            "HAZARD_GUARD_CLOUD_LINEAR_UPDATE", "0.10"
        ),
        "cloud_angular_update": os.getenv(
            "HAZARD_GUARD_CLOUD_ANGULAR_UPDATE", "0.10472"
        ),
    }
    rtabmap_arguments = {
        "database_path": database_path,
        "storage_path": storage_path,
        "cloud_stamp_mode": LaunchConfiguration("cloud_stamp_mode"),
        "cloud_stamp_offset_sec": LaunchConfiguration(
            "cloud_stamp_offset_sec"
        ),
        "sync_diagnostics": LaunchConfiguration("sync_diagnostics"),
        "rtabmap_registration_strategy": LaunchConfiguration(
            "rtabmap_registration_strategy"
        ),
        "cloud_fixed_frame": LaunchConfiguration("cloud_fixed_frame"),
        "cloud_output_frame": LaunchConfiguration("cloud_output_frame"),
        "optimized_cloud": LaunchConfiguration("optimized_cloud"),
        **cloud_arguments,
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=str(
                    workspace
                    / "runtime"
                    / "maps"
                    / "physical_rtabmap.db"
                ),
            ),
            DeclareLaunchArgument(
                "storage_path",
                default_value=str(workspace / "runtime" / "maps"),
            ),
            DeclareLaunchArgument("enable_rtabmap", default_value="true"),
            DeclareLaunchArgument(
                "enable_thermal_mapping",
                default_value=os.getenv(
                    "HAZARD_GUARD_THERMAL_MAPPING_ENABLED", "true"
                ),
                description=(
                    "Publish the calibrated physical thermal 3D cloud when "
                    "the RGB-D mapping profile is active"
                ),
            ),
            DeclareLaunchArgument("start_thermal_camera", default_value="true"),
            DeclareLaunchArgument(
                "thermal_output_topic",
                default_value="/hazard_guard/thermal/points",
            ),
            DeclareLaunchArgument("thermal_map_frame", default_value="map"),
            DeclareLaunchArgument("thermal_min_temp_c", default_value="10.0"),
            DeclareLaunchArgument("thermal_max_temp_c", default_value="60.0"),
            DeclareLaunchArgument("thermal_voxel_size", default_value="0.05"),
            DeclareLaunchArgument(
                "cloud_normal_points",
                default_value=cloud_defaults["cloud_normal_points"],
            ),
            DeclareLaunchArgument(
                "cloud_high_load_points",
                default_value=cloud_defaults["cloud_high_load_points"],
            ),
            DeclareLaunchArgument(
                "cloud_normal_input_hz",
                default_value=cloud_defaults["cloud_normal_input_hz"],
            ),
            DeclareLaunchArgument(
                "cloud_high_load_input_hz",
                default_value=cloud_defaults["cloud_high_load_input_hz"],
            ),
            DeclareLaunchArgument(
                "cloud_normal_surface_hz",
                default_value=cloud_defaults["cloud_normal_surface_hz"],
            ),
            DeclareLaunchArgument(
                "cloud_high_load_surface_hz",
                default_value=cloud_defaults["cloud_high_load_surface_hz"],
            ),
            DeclareLaunchArgument(
                "cloud_decimation",
                default_value=cloud_defaults["cloud_decimation"],
            ),
            DeclareLaunchArgument(
                "cloud_voxel_size",
                default_value=cloud_defaults["cloud_voxel_size"],
            ),
            DeclareLaunchArgument(
                "cloud_linear_update",
                default_value=cloud_defaults["cloud_linear_update"],
            ),
            DeclareLaunchArgument(
                "cloud_angular_update",
                default_value=cloud_defaults["cloud_angular_update"],
            ),
            DeclareLaunchArgument(
                "cloud_stamp_mode",
                default_value="latest",
                choices=["preserve", "offset", "latest"],
            ),
            DeclareLaunchArgument(
                "cloud_stamp_offset_sec",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "sync_diagnostics",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "rtabmap_registration_strategy",
                default_value="1",
                choices=["0", "1", "2"],
            ),
            DeclareLaunchArgument(
                "cloud_fixed_frame",
                default_value="odom",
                choices=["odom", "map"],
            ),
            DeclareLaunchArgument(
                "cloud_output_frame",
                default_value="odom",
                choices=["odom", "map"],
            ),
            DeclareLaunchArgument(
                "optimized_cloud",
                default_value="false",
            ),
            # The vendor top-level launch does not forward this argument,
            # while its nested SLAM launch reads the global configuration.
            SetLaunchConfiguration("use_sim_time", "false"),
            include("yahboomcar_nav", "map_slam_toolbox_launch.py"),
            include(
                "ascamera",
                "hp60c.launch.py",
                condition=IfCondition(enable_rtabmap),
            ),
            include(
                "hazard_guard_simulation",
                "rtabmap_real.launch.py",
                rtabmap_arguments,
                condition=IfCondition(enable_rtabmap),
            ),
            include(
                "hazard_guard_simulation",
                "physical_thermal_camera.launch.py",
                {"show_gui": "false"},
                condition=thermal_camera_condition,
            ),
            Node(
                package="hazard_guard_simulation",
                executable="thermal_cloud.py",
                name="physical_thermal_cloud",
                output="screen",
                condition=thermal_mapping_condition,
                parameters=[
                    {
                        "use_sim_time": False,
                        "depth_image": (
                            "/ascamera_hp60c/camera_publisher/depth0/image_raw"
                        ),
                        "depth_info": (
                            "/ascamera_hp60c/camera_publisher/depth0/camera_info"
                        ),
                        "thermal_image": "/thermal_camera/image_raw",
                        "thermal_info": "/thermal_camera/camera_info",
                        "output_topic": LaunchConfiguration(
                            "thermal_output_topic"
                        ),
                        "map_frame": LaunchConfiguration("thermal_map_frame"),
                        # 0 selects 0.001 automatically for HP60C 16UC1.
                        "depth_scale": 0.0,
                        # Keep raw vendor stamps but pair the independently
                        # clocked physical cameras by local receipt time.
                        "sync_by_receipt_time": True,
                        "min_temp_c": ParameterValue(
                            LaunchConfiguration("thermal_min_temp_c"),
                            value_type=float,
                        ),
                        "max_temp_c": ParameterValue(
                            LaunchConfiguration("thermal_max_temp_c"),
                            value_type=float,
                        ),
                        "voxel_size": ParameterValue(
                            LaunchConfiguration("thermal_voxel_size"),
                            value_type=float,
                        ),
                        # Bound the secondary visualization map so it cannot
                        # starve SLAM Toolbox/Nav2 on the Jetson.
                        "max_voxels": 120000,
                        "stride": 4,
                        "publish_period_sec": 1.0,
                    }
                ],
            ),
        ]
    )
