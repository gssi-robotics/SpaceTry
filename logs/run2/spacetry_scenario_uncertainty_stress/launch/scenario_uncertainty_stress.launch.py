import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_waypoint(path: str, name: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return ((data.get("waypoints") or {}).get(name) or {})


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("spacetry_bringup")
    mission_waypoints = os.path.join(
        get_package_share_directory("spacetry_mission"),
        "config",
        "waypoints.yaml",
    )
    outpost = _load_waypoint(mission_waypoints, "outpost_habitat_01")
    science_rock = _load_waypoint(mission_waypoints, "science_rock_01")

    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")
    report_dir = LaunchConfiguration("report_dir")
    scenario_config = LaunchConfiguration("scenario_config")
    contract_path = LaunchConfiguration("contract_path")

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "battery": battery,
            "enable_demo_nodes": "false",
            "enable_bt_runner": "true",
            "tree_file": os.path.join(
                get_package_share_directory("spacetry_bt"),
                "trees",
                "base_bt.xml",
            ),
        }.items(),
    )

    driver = Node(
        package="spacetry_scenario_uncertainty_stress",
        executable="scenario_driver",
        name="scenario_driver",
        output="screen",
        parameters=[
            scenario_config,
            {
                "use_sim_time": True,
                "report_dir": report_dir,
                "contract_path": contract_path,
                "science_goal_x": float(science_rock.get("x", 50.1)),
                "science_goal_y": float(science_rock.get("y", -80.0)),
                "science_goal_tolerance_m": float(science_rock.get("tolerance", 2.0)),
                "outpost_x": float(outpost.get("x", 66.0)),
                "outpost_y": float(outpost.get("y", 0.0)),
                "outpost_tolerance_m": float(outpost.get("tolerance", 2.0)),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="1"),
            DeclareLaunchArgument("battery", default_value="1.0"),
            DeclareLaunchArgument(
                "report_dir",
                default_value="/tmp/spacetry_scenario_uncertainty_stress",
            ),
            DeclareLaunchArgument(
                "scenario_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("spacetry_scenario_uncertainty_stress"),
                        "config",
                        "scenario_uncertainty_stress.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "contract_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("spacetry_scenario_uncertainty_stress"),
                        "config",
                        "scenario_contract.yaml",
                    ]
                ),
            ),
            baseline,
            driver,
        ]
    )
