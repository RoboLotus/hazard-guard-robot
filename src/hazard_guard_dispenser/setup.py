from glob import glob

from setuptools import find_packages, setup
package_name = "hazard_guard_dispenser"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="sumin",
    maintainer_email="sumin@example.com",
    description="Beacon cube dispenser servo control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dispenser_node = hazard_guard_dispenser.node:main",
        ]
    },
)
