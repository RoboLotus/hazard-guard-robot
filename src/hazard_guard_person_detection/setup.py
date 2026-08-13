from glob import glob
from setuptools import find_packages, setup


package_name = "hazard_guard_person_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HazardGuard Team",
    maintainer_email="maintainers@hazardguard.local",
    description="YOLO person detection and RGB-D distance estimation for HazardGuard.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_detection_node = hazard_guard_person_detection.node:main",
        ],
    },
)
