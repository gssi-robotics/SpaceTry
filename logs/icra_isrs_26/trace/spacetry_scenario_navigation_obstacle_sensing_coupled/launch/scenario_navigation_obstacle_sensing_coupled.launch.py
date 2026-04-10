from datetime import datetime, timezone
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_name = "spacetry_scenario_navigation_obstacle_sensing_coupled"
    pkg_share = get_package_share_directory(scenario_name)
    bringup_share = get_package_share_directory("spacetry_bringup")
    bt_share = get_package_share_directory("spacetry_bt")
    scenario_config_file = os.path.join(pkg_share, "config", "scenario_config.yaml")
    scenario_contract_file = os.path.join(pkg_share, "config", "scenario_contract.yaml")
    baseline_tree_file = os.path.join(bt_share, "trees", "base_bt.xml")

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_root_dir = "/ws/log"
    scenario_log_dir = os.path.join(log_root_dir, scenario_name)
    rosbag_output_dir = os.path.join(scenario_log_dir, "rosbags", run_id)

    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")
    enable_rosbag = LaunchConfiguration("enable_rosbag")

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
        package=scenario_name,
        executable="scenario_driver",
        name="scenario_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scenario_name": scenario_name,
                "scenario_config_file": scenario_config_file,
                "scenario_contract_file": scenario_contract_file,
                "log_root_dir": log_root_dir,
                "rosbag_output_dir": rosbag_output_dir,
                "run_id": run_id,
            }
        ],
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            rosbag_output_dir,
            "/clock",
            "/scan",
            "/cmd_vel",
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
        ],
        condition=IfCondition(enable_rosbag),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="1",
                description="Run Gazebo in headless mode for scenario execution.",
            ),
            DeclareLaunchArgument(
                "battery",
                default_value="0.79",
                description="Scenario battery SOC chosen to avoid unrelated MR_011 dominance.",
            ),
            DeclareLaunchArgument(
                "enable_rosbag",
                default_value="true",
                description="Record scenario rosbag topics under /ws/log.",
            ),
            baseline_bringup,
            scenario_driver,
            TimerAction(period=5.0, actions=[rosbag_record]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=scenario_driver,
                    on_exit=[Shutdown(reason="scenario driver finished")],
                )
            ),
        ]
    )
