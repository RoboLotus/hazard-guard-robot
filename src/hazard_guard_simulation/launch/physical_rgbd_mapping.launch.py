"""Field-tested M1 localization plus second-pass HP60C RGB-D collection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(
        get_package_share_directory("hazard_guard_simulation")
    )
    feature_defaults = {
        "use_person_safety": "false",
        "start_person_camera": "true",
        "person_model_path": "yolo11n.pt",
        "person_device": "",
        "person_confidence": "0.4",
        "person_image_size": "640",
        "person_inference_rate_hz": "10.0",
        "person_depth_registration_verified": "false",
        "enable_thermal_pipeline": "false",
        "thermal_roi_config": "",
        "thermal_history_path": (
            "~/.local/share/hazard_guard/thermal_history.jsonl"
        ),
        "thermal_image_topic": "/thermal_camera/image_raw",
        "thermal_info_topic": "/thermal_camera/camera_info",
        "thermal_depth_image_topic": "/depth_camera/image_raw",
        "thermal_depth_info_topic": "/depth_camera/camera_info",
        "thermal_scale": "1.0",
        "thermal_offset_c": "0.0",
    }
    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "map",
            "initial_pose_x",
            "initial_pose_y",
            "initial_pose_yaw",
            "rtabmap_database_path",
            "rtabmap_storage_path",
            "rgbd_cloud_stamp_mode",
            "rgbd_cloud_stamp_offset_sec",
            *feature_defaults,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_database_path"),
            DeclareLaunchArgument("rtabmap_storage_path"),
            DeclareLaunchArgument(
                "rgbd_cloud_stamp_mode",
                default_value="offset",
                choices=["preserve", "offset"],
            ),
            DeclareLaunchArgument(
                "rgbd_cloud_stamp_offset_sec",
                default_value="0.0",
            ),
            *(
                DeclareLaunchArgument(name, default_value=default)
                for name, default in feature_defaults.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        simulation_share
                        / "launch"
                        / "physical_patrol.launch.py"
                    )
                ),
                launch_arguments={
                    **forwarded,
                    "enable_rgbd_mapping": "true",
                }.items(),
            ),
        ]
    )
