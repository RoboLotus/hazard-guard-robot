from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "hazard_guard_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kohs2k21",
    maintainer_email="gw010101@naver.com",
    description="Launch and configuration package for HazardGuard development.",
    license="Apache-2.0",
)
