import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scenario_share = get_package_share_directory(
        "spacetry_scenario_navigation_obstacle_degraded_perception"
    )
    bringup_share = get_package_share_directory("spacetry_bringup")
    bt_share = get_package_share_directory("spacetry_bt")

    scenario_config = os.path.join(scenario_share, "config", "scenario_config.yaml")
    scenario_contract = os.path.join(scenario_share, "config", "scenario_contract.yaml")
    baseline_tree_file = os.path.join(bt_share, "trees", "base_bt.xml")

    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")
    output_root = LaunchConfiguration("output_root")
    rosbag_enabled = LaunchConfiguration("rosbag_enabled")

    baseline_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "battery": battery,
            "tree_file": baseline_tree_file,
        }.items(),
    )

    scenario_driver = Node(
        package="spacetry_scenario_navigation_obstacle_degraded_perception",
        executable="scenario_driver",
        name="scenario_navigation_obstacle_degraded_perception_driver",
        output="screen",
        parameters=[
            {
                "scenario_contract_file": ParameterValue(
                    scenario_contract, value_type=str
                ),
                "scenario_config_file": ParameterValue(
                    scenario_config, value_type=str
                ),
                "output_root": ParameterValue(output_root, value_type=str),
                "rosbag_enabled": ParameterValue(rosbag_enabled, value_type=bool),
            }
        ],
    )

    shutdown_on_driver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=scenario_driver,
            on_exit=[EmitEvent(event=Shutdown(reason="scenario driver completed"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="1"),
            DeclareLaunchArgument("battery", default_value="0.79"),
            DeclareLaunchArgument("output_root", default_value="/ws/log"),
            DeclareLaunchArgument("rosbag_enabled", default_value="true"),
            baseline_bringup,
            scenario_driver,
            shutdown_on_driver_exit,
        ]
    )
