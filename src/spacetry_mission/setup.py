from setuptools import setup
import os
from glob import glob

package_name = "marti_mission"

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
    maintainer="MARTI",
    description="Mission config services for MARTI",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "marti_mission_server = marti_mission.marti_mission_server:main",
        ],
    },
)