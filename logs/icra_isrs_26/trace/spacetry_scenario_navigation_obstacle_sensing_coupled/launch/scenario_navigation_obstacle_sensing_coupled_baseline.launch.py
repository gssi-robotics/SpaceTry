import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    scenario_name = "spacetry_scenario_navigation_obstacle_sensing_coupled"
    pkg_share = get_package_share_directory(scenario_name)
    baseline_config_file = os.path.join(
        pkg_share,
        "config",
        "scenario_config_baseline.yaml",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_share,
                        "launch",
                        "scenario_navigation_obstacle_sensing_coupled.launch.py",
                    )
                ),
                launch_arguments={
                    "headless": "1",
                    "battery": "0.79",
                    "enable_rosbag": "true",
                    "scenario_config_file": baseline_config_file,
                }.items(),
            )
        ]
    )
