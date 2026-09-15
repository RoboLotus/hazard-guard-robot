import os
from glob import glob
from setuptools import setup

package_name = 'hazard_guard_iot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboLotus',
    maintainer_email='dev@robolotus.local',
    description='HazardGuard IoT bridge (ROS 2 to MQTT)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = hazard_guard_iot.mqtt_bridge_node:main',
            'system_monitor_node = hazard_guard_iot.system_monitor_node:main',
            'status_server = hazard_guard_iot.status_server:main',
        ],
    },
)
