from setuptools import find_packages, setup

package_name = "hazard_guard_mission_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="ROS 2 action server that executes HazardGuard patrol missions.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_manager = hazard_guard_mission_manager.node:main",
        ]
    },
)
