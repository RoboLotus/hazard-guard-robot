"""Start RTAB-Map only after saved-map localization is demonstrably ready."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _after_gate_exit(event, context, sim_capture, real_capture):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        reason = (
            "Localization readiness validation failed "
            f"(exit code {event.returncode}); RTAB-Map was not started"
        )
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]
    backend = LaunchConfiguration("backend").perform(context)
    capture = sim_capture if backend == "sim" else real_capture
    return [LogInfo(msg=f"Starting {backend} RGB-D capture"), capture]


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_path = LaunchConfiguration("map")
    database_path = LaunchConfiguration("database_path")
    reset_database = LaunchConfiguration("reset_database")
    storage_path = LaunchConfiguration("storage_path")

    gate = Node(
        package="hazard_guard_simulation",
        executable="localization_ready_gate.py",
        name="localization_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_path": map_path,
                "global_frame": "map",
                "base_frame": "base_footprint",
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("readiness_timeout_sec"),
                    value_type=float,
                ),
                "stable_samples": ParameterValue(
                    LaunchConfiguration("readiness_stable_samples"),
                    value_type=int,
                ),
                "max_tf_age_sec": ParameterValue(
                    LaunchConfiguration("readiness_max_tf_age_sec"),
                    value_type=float,
                ),
            }
        ],
    )

    sim_capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "rtabmap_sim.launch.py")
        ),
        launch_arguments={
            "start_simulation": "false",
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "demo_route": "false",
            "database_path": database_path,
            "reset_database": reset_database,
            "publish_tf": "false",
            "map_frame_id": "map",
            "map_topic": "/rtabmap/grid_map",
            "optimized_cloud": "true",
            "parameters_file": str(
                simulation_share / "config" / "rtabmap_rgbd_capture.yaml"
            ),
        }.items(),
    )
    real_capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "rtabmap_real.launch.py")
        ),
        launch_arguments={
            "database_path": database_path,
            "reset_database": reset_database,
            "storage_path": storage_path,
            "cloud_stamp_mode": LaunchConfiguration("cloud_stamp_mode"),
            "cloud_stamp_offset_sec": LaunchConfiguration(
                "cloud_stamp_offset_sec"
            ),
            "cloud_fixed_frame": "map",
            "cloud_output_frame": "map",
            "optimized_cloud": "true",
            "odom_frame_id": "map",
            "map_frame_id": "map",
            "rtabmap_registration_strategy": "0",
            "subscribe_scan": "false",
            "neighbor_link_refining": "false",
            "proximity_by_space": "false",
            "loop_closure_threshold": "1.0",
            "optimize_max_error": "3.0",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", choices=["sim", "real"]),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("database_path"),
            DeclareLaunchArgument("reset_database", default_value="false"),
            DeclareLaunchArgument("storage_path", default_value="/tmp"),
            DeclareLaunchArgument("cloud_stamp_mode", default_value="offset"),
            DeclareLaunchArgument(
                "cloud_stamp_offset_sec", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "readiness_timeout_sec", default_value="60.0"
            ),
            DeclareLaunchArgument(
                "readiness_stable_samples", default_value="3"
            ),
            DeclareLaunchArgument(
                "readiness_max_tf_age_sec", default_value="2.0"
            ),
            gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gate,
                    on_exit=lambda event, context: _after_gate_exit(
                        event,
                        context,
                        sim_capture,
                        real_capture,
                    ),
                )
            ),
        ]
    )
