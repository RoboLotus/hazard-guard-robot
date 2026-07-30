from setuptools import find_packages, setup

package_name = "hazard_guard_mock_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Hardware-free HazardGuard robot telemetry and command simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "initial_pose_once = hazard_guard_mock_robot.initial_pose_once:main",
            "mock_robot = hazard_guard_mock_robot.node:main",
            "nav2_smoke_test = hazard_guard_mock_robot.nav2_smoke_test:main",
            "thermal_detector_mock = hazard_guard_mock_robot.thermal_detector_mock:main",
        ]
    },
)
