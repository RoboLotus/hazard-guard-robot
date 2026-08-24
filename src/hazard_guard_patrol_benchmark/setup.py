from glob import glob

from setuptools import find_packages, setup


package_name = "hazard_guard_patrol_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/scripts",
            glob("scripts/*.py") + glob("scripts/*.ps1"),
        ),
        (f"share/{package_name}/scripts/docker", glob("scripts/docker/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Gazebo-only patrol coverage and timing benchmark.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "patrol_benchmark = hazard_guard_patrol_benchmark.node:main",
            "patrol_benchmark_run = hazard_guard_patrol_benchmark.runner:main",
            "patrol_benchmark_aggregate = hazard_guard_patrol_benchmark.aggregate:main",
            "docker_profile_probe = hazard_guard_patrol_benchmark.profile_probe:main",
            "docker_profile_select = hazard_guard_patrol_benchmark.profile_select:main",
        ]
    },
)
