from setuptools import find_packages, setup


package_name = "hazard_guard_sensor_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Shared sensor profiles for HazardGuard ROS 2 packages.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
