"""Serial-port half of the always-on IoT stack: motor board driver + battery.

Started by hazardguard-base.service so the app shows battery while the robot
is idle. physical_m1_bringup.launch.py stops that service before starting its
own copy of these two nodes, because /dev/myserial can only be opened once.
Battery thresholds mirror physical_m1_bringup.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_bringup',
            executable='Mcnamu_driver_M1',
            name='driver_node',
        ),
        Node(
            package='hazard_guard_robot_telemetry',
            executable='battery_telemetry',
            name='hazard_guard_battery_telemetry',
            output='screen',
            parameters=[{
                'input_topic': '/voltage',
                'output_topic': '/hazard_guard/battery',
                'window_size': 20,
                'empty_voltage_v': 10.5,
                'full_voltage_v': 12.6,
            }],
        ),
    ])
