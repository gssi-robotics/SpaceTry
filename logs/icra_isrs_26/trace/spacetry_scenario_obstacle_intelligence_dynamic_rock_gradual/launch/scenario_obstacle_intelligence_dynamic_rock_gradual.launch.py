import os
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "spacetry_scenario_obstacle_intelligence_dynamic_rock_gradual"
    scenario_share = get_package_share_directory(package_name)
    bringup_share = get_package_share_directory("spacetry_bringup")
    bt_share = get_package_share_directory("spacetry_bt")

    run_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_root = os.path.join("/ws", "log", "scenario_obstacle_intelligence_dynamic_rock_gradual")
    bag_dir = os.path.join(
        output_root,
        "rosbags",
        f"scenario_obstacle_intelligence_dynamic_rock_gradual_{run_stamp}",
    )
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    os.makedirs(os.path.join(output_root, "runtime"), exist_ok=True)
    os.makedirs(os.path.join(output_root, "metrics"), exist_ok=True)

    config_file = os.path.join(
        scenario_share,
        "config",
        "scenario_obstacle_intelligence_dynamic_rock_gradual.yaml",
    )
    contract_file = os.path.join(
        scenario_share,
        "config",
        "scenario_obstacle_intelligence_dynamic_rock_gradual_contract.yaml",
    )
    tree_file = os.path.join(bt_share, "trees", "base_bt.xml")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "battery": LaunchConfiguration("battery"),
            "spawn_waypoint": "dock_pad_01",
            "tree_file": tree_file,
        }.items(),
    )

    rosbag_topics = [
        "/mobile_base_controller/odom",
        "/scan",
        "/cmd_vel",
        "/battery/soc",
        "/battery/near_outpost",
        "/obstacle/front",
        "/obstacle/left",
        "/obstacle/right",
        "/monitor/events",
        "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
        "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
    ]

    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_dir, *rosbag_topics],
        output="screen",
    )

    scenario_node = Node(
        package=package_name,
        executable="scenario_driver_node",
        name="scenario_obstacle_intelligence_dynamic_rock_gradual",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": True,
                "config_file": config_file,
                "contract_file": contract_file,
                "output_root": output_root,
                "run_stamp": run_stamp,
            },
        ],
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=scenario_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="scenario_obstacle_intelligence_dynamic_rock_gradual completed"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="1"),
            DeclareLaunchArgument("battery", default_value="1.0"),
            rosbag_record,
            bringup_launch,
            scenario_node,
            shutdown_on_exit,
        ]
    )
