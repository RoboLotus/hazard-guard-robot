from setuptools import find_packages, setup


package_name = "hazard_guard_bag_recorder"


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
        (f"share/{package_name}/config", ["config/profiles.json"]),
        (f"share/{package_name}/launch", ["launch/bag_record.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Profiled and validated rosbag2 recorder for HazardGuard.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bag_session_manager = hazard_guard_bag_recorder.node:main",
        ]
    },
)
