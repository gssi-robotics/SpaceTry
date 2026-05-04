from pathlib import Path

from setuptools import setup

package_name = "spacetry_scenario_perception_lidar_degradation_gradual"

Path("resource").mkdir(exist_ok=True)
(Path("resource") / package_name).touch(exist_ok=True)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/scenario_perception_lidar_degradation_gradual.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/scenario_config.yaml",
            "config/scenario_contract.yaml",
        ]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="SpaceTry Team",
    maintainer_email="keila.lima@gssi.it",
    description="Autonomy scenario driver for runtime LiDAR-derived obstacle classification degradation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_lidar_degradation_driver = "
            "spacetry_scenario_perception_lidar_degradation_gradual.driver:main",
        ],
    },
)
