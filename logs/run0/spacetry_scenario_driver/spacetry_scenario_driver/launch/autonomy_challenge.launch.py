import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_config = LaunchConfiguration("scenario_config")
    battery = LaunchConfiguration("battery")
    headless = LaunchConfiguration("headless")

    bringup_launch = os.path.join(
        get_package_share_directory("spacetry_bringup"),
        "launch",
        "spacetry_curiosity_outpost.launch.py",
    )

    bt_tree = PathJoinSubstitution([FindPackageShare("spacetry_bt"), "trees", "base_bt.xml"])
    bt_params = PathJoinSubstitution([FindPackageShare("spacetry_bt"), "bt_params.yaml"])
    default_scenario = PathJoinSubstitution(
        [FindPackageShare("spacetry_scenario_driver"), "config", "autonomy_challenge.yaml"]
    )

    bt_runner = Node(
        package="spacetry_bt",
        executable="spacetry_bt_runner",
        name="spacetry_bt_runner",
        output="screen",
        parameters=[
            bt_params,
            {
                "tree_file": bt_tree,
                "tick_hz": 10.0,
                "use_sim_time": True,
                "waypoints.science_rock": [50.10, -80.0, 3.07],
                "waypoints.outpost": [66.0, 0.0, 3.07],
            },
        ],
    )

    scenario_driver = Node(
        package="spacetry_scenario_driver",
        executable="scenario_driver",
        name="scenario_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scenario_file": scenario_config,
            }
        ],
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "headless": headless,
            "battery": battery,
            "scenario_mode": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_config",
                default_value=default_scenario,
                description="Scenario driver YAML configuration.",
            ),
            DeclareLaunchArgument(
                "battery",
                default_value="0.55",
                description="Initial battery state-of-charge for the scenario launch.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="0",
                description="Launch Gazebo headless if set to 1.",
            ),
            bringup,
            bt_runner,
            scenario_driver,
        ]
    )
