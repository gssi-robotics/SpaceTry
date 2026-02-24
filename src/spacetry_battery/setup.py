from setuptools import setup

package_name = "spacetry_battery"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/battery_manager.yaml"]),
        ("share/" + package_name + "/launch", ["launch/battery_manager.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spacetry",
    maintainer_email="noreply@example.com",
    description="Standalone battery manager node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "battery_manager = spacetry_battery.battery_manager_node:main",
        ],
    },
)
