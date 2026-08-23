from setuptools import find_packages, setup


package_name = "hazard_guard_performance_monitor"

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
        (f"share/{package_name}/config", ["config/performance_monitor.yaml"]),
        (f"share/{package_name}/launch", ["launch/performance_monitor.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Mission-scoped Jetson and Linux performance profiler.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "performance_monitor = hazard_guard_performance_monitor.node:main",
        ]
    },
)
