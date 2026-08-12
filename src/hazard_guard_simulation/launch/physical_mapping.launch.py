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
from launch.substitutions import LaunchConfiguration


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
                {
                    "database_path": database_path,
                    "storage_path": storage_path,
                    **cloud_arguments,
                },
                condition=IfCondition(enable_rtabmap),
            ),
        ]
    )
