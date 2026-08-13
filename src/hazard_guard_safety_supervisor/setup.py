import os
from glob import glob

from setuptools import find_packages, setup


package_name = "hazard_guard_safety_supervisor"


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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Person proximity safety state supervisor for HazardGuard.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "person_safety_supervisor = hazard_guard_safety_supervisor.node:main",
        ]
    },
)
