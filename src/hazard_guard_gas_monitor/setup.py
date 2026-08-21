from setuptools import find_packages, setup


package_name = "hazard_guard_gas_monitor"

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
        (f"share/{package_name}/config", ["config/demo_gas_scenario.json"]),
        (f"share/{package_name}/launch", ["launch/gas_simulation.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="beomaura",
    maintainer_email="junbeom@dgu.ac.kr",
    description="VOC, CO and CO2 early-warning simulation for HazardGuard.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gas_sensor_simulator = hazard_guard_gas_monitor.simulator_node:main",
            "gas_detector = hazard_guard_gas_monitor.detector_node:main",
        ]
    },
)
