from glob import glob
import os

from setuptools import setup

package_name = "spacetry_scenario_uncertainty_stress"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="ricardo.caldas@gssi.it",
    description="Scenario driver package for uncertainty stress tests.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scenario_driver = spacetry_scenario_uncertainty_stress.scenario_driver_node:main",
        ],
    },
)
