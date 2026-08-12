from glob import glob
import os

from setuptools import find_packages, setup


package_name = "hazard_guard_thermal_analysis"

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
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboLotus",
    maintainer_email="junbeom@dgu.ac.kr",
    description=(
        "Shared thermal-depth fusion and voxel analysis for simulated and "
        "physical HazardGuard robots."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thermal_depth_fusion = "
            "hazard_guard_thermal_analysis.fusion_node:main",
            "thermal_voxel_analyzer = "
            "hazard_guard_thermal_analysis.analyzer_node:main",
        ]
    },
)
