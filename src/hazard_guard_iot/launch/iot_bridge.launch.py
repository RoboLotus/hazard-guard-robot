import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hazard_guard_iot')
    default_config = os.path.join(share, 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('broker_host', default_value='localhost'),
        DeclareLaunchArgument('robot_id', default_value='robot01'),
        DeclareLaunchArgument('system_monitor', default_value='true'),
        Node(
            package='hazard_guard_iot',
            executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'broker_host': LaunchConfiguration('broker_host'),
                'robot_id': LaunchConfiguration('robot_id'),
            }],
        ),
        Node(
            package='hazard_guard_iot',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('system_monitor')),
        ),
    ])
