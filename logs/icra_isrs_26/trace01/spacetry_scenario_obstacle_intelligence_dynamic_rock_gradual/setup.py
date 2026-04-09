from setuptools import setup

package_name = "spacetry_scenario_obstacle_intelligence_dynamic_rock_gradual"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/scenario_obstacle_intelligence_dynamic_rock_gradual.yaml",
                "config/scenario_obstacle_intelligence_dynamic_rock_gradual_contract.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/scenario_obstacle_intelligence_dynamic_rock_gradual.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="ricardo.caldas@gssi.it",
    description="Runtime obstacle-injection scenario driver for autonomy evaluation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scenario_driver_node = spacetry_scenario_obstacle_intelligence_dynamic_rock_gradual.scenario_driver_node:main",
        ],
    },
)
