import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "spacetry_scenario_obstacle_sensing_stress"
    scenario_share = get_package_share_directory(package_name)
    bringup_share = get_package_share_directory("spacetry_bringup")
    bt_share = get_package_share_directory("spacetry_bt")

    contract_file = os.path.join(scenario_share, "config", "scenario_contract.yaml")
    config_file = os.path.join(scenario_share, "config", "scenario_config.yaml")
    tree_file = os.path.join(bt_share, "trees", "base_bt.xml")

    output_root = LaunchConfiguration("output_root")
    run_timeout_s = LaunchConfiguration("run_timeout_s")
    battery = LaunchConfiguration("battery")
    headless = LaunchConfiguration("headless")
    record_bag = LaunchConfiguration("record_bag")
    rosbag_output = LaunchConfiguration("rosbag_output")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "battery": battery,
            "tree_file": tree_file,
            "spawn_waypoint": "dock_pad_01",
            "enable_demo_nodes": "false",
            "enable_bt_runner": "true",
        }.items(),
    )

    scenario_driver = Node(
        package=package_name,
        executable="scenario_driver",
        name="scenario_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scenario_contract_file": contract_file,
                "scenario_config_file": config_file,
                "output_root": output_root,
                "run_timeout_s": run_timeout_s,
                "goal_name": "science_rock_01",
                "goal_x": 50.10,
                "goal_y": -80.0,
                "goal_tolerance_m": 1.5,
            }
        ],
    )

    rosbag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            rosbag_output,
            "/clock",
            "/cmd_vel",
            "/scan",
            "/mobile_base_controller/odom",
            "/obstacle/front",
            "/obstacle/left",
            "/obstacle/right",
            "/obstacle/state",
            "/battery/soc",
            "/battery/near_outpost",
            "/monitor/events",
            "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
            "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
            "/at_science_rock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="1"),
            DeclareLaunchArgument(
                "battery",
                default_value="0.79",
                description=(
                    "Initial SOC kept below the MR_011 full-battery guard so the scenario isolates "
                    "navigation and perception stress instead of the baseline speed monitor mismatch."
                ),
            ),
            DeclareLaunchArgument("run_timeout_s", default_value="1200.0"),
            DeclareLaunchArgument(
                "output_root",
                default_value="/ws/log/scenario_obstacle_sensing_stress",
            ),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument(
                "rosbag_output",
                default_value="/ws/log/scenario_obstacle_sensing_stress/rosbags/scenario_obstacle_sensing_stress_bag",
            ),
            bringup,
            rosbag_record,
            scenario_driver,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=scenario_driver,
                    on_exit=[EmitEvent(event=Shutdown(reason="Scenario driver finished"))],
                )
            ),
        ]
    )
