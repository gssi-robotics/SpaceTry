from pathlib import Path

from setuptools import setup

package_name = "spacetry_scenario_obstacle_sensing_stress"

Path("resource").mkdir(exist_ok=True)
(Path("resource") / package_name).touch(exist_ok=True)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/scenario_config.yaml", "config/scenario_contract.yaml"]),
        ("share/" + package_name + "/launch", ["launch/scenario_obstacle_sensing_stress.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="ricardo.caldas@gssi.it",
    description="Scenario driver for runtime obstacle blocking with degraded obstacle interpretation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scenario_driver = spacetry_scenario_obstacle_sensing_stress.scenario_driver_node:main",
        ],
    },
)
