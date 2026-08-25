from setuptools import find_packages, setup


package_name = "hazard_guard_robot_telemetry"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Physical ROSMASTER M1 battery telemetry adapter.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "battery_telemetry = hazard_guard_robot_telemetry.node:main",
        ],
    },
)
