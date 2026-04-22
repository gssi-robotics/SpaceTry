from pathlib import Path

from setuptools import setup


package_name = "spacetry_scenario_navigation_obstacle_degraded_perception"

Path("resource").mkdir(exist_ok=True)
(Path("resource") / package_name).touch(exist_ok=True)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/scenario_config.yaml",
            "config/scenario_contract.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/scenario_navigation_obstacle_degraded_perception.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="ricardo.caldas@gssi.it",
    description="Scenario driver for navigation obstacle blocking with degraded perception.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario_driver = "
            "spacetry_scenario_navigation_obstacle_degraded_perception.scenario_driver_node:main",
        ],
    },
)
