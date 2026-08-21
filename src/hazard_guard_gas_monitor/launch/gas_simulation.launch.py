from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("hazard_guard_gas_monitor"))
    default_scenario = share / "config" / "demo_gas_scenario.json"
    use_sim_time = LaunchConfiguration("use_sim_time")
    scenario_path = LaunchConfiguration("scenario_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("scenario_path", default_value=str(default_scenario)),
            Node(
                package="hazard_guard_gas_monitor",
                executable="gas_sensor_simulator",
                name="hazard_guard_gas_sensor_simulator",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "scenario_path": scenario_path}],
            ),
            Node(
                package="hazard_guard_gas_monitor",
                executable="gas_detector",
                name="hazard_guard_gas_detector",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "scenario_path": scenario_path}],
            ),
        ]
    )
