from glob import glob
import os

from setuptools import setup

package_name = "spacetry_scenario_navigation_obstacle_sensing_coupled"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="keila.lima@gssi.it",
    description="Scenario driver for coupled runtime obstacle blocking and degraded obstacle interpretation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario_driver = spacetry_scenario_navigation_obstacle_sensing_coupled.scenario_driver_node:main",
        ],
    },
)
