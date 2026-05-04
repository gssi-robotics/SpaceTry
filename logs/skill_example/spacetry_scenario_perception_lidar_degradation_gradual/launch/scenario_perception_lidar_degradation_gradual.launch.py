import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    scenario_share = get_package_share_directory(
        "spacetry_scenario_perception_lidar_degradation_gradual"
    )
    bringup_share = get_package_share_directory("spacetry_bringup")

    output_root = LaunchConfiguration("output_root")
    run_label = LaunchConfiguration("run_label")
    record_rosbag = LaunchConfiguration("record_rosbag")
    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")

    scenario_config_file = os.path.join(scenario_share, "config", "scenario_config.yaml")
    scenario_contract_file = os.path.join(scenario_share, "config", "scenario_contract.yaml")

    baseline_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "battery": battery,
            "enable_bt_runner": "true",
            "enable_demo_nodes": "false",
        }.items(),
    )

    scenario_driver = Node(
        package="spacetry_scenario_perception_lidar_degradation_gradual",
        executable="perception_lidar_degradation_driver",
        name="perception_lidar_degradation_driver",
        output="screen",
        parameters=[
            {
                "scenario_config_file": scenario_config_file,
                "scenario_contract_file": scenario_contract_file,
                "output_root": output_root,
                "run_label": run_label,
            }
        ],
    )

    rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            PathJoinSubstitution([output_root, "rosbags", run_label]),
            "/clock",
            "/mobile_base_controller/odom",
            "/cmd_vel",
            "/scan",
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
        output="screen",
        condition=IfCondition(record_rosbag),
    )

    shutdown_when_driver_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=scenario_driver,
            on_exit=[EmitEvent(event=Shutdown(reason="scenario driver exited"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="1"),
            DeclareLaunchArgument("battery", default_value="1.0"),
            DeclareLaunchArgument(
                "output_root",
                default_value="/ws/logs/skill_example",
            ),
            DeclareLaunchArgument("run_label", default_value="full_run"),
            DeclareLaunchArgument("record_rosbag", default_value="true"),
            SetParameter(name="use_sim_time", value=True),
            baseline_bringup,
            rosbag,
            scenario_driver,
            shutdown_when_driver_exits,
        ]
    )
