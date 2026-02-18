from setuptools import setup
import os
from glob import glob

package_name = "spacetry_mission"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="SpaceTry 🥐",
    description="Mission config services for SpaceTry 🥐",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "spacetry_mission_server = spacetry_mission.spacetry_mission_server:main",
        ],
    },
)