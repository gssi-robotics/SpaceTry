import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")
    report_path = LaunchConfiguration("report_path")
    scenario_config = LaunchConfiguration("scenario_config")

    bringup_share = get_package_share_directory("spacetry_bringup")
    bt_share = get_package_share_directory("spacetry_bt")
    mission_share = get_package_share_directory("spacetry_mission")
    scenario_share = get_package_share_directory("spacetry_scenario_driver")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "spacetry_curiosity_outpost.launch.py")
        ),
        launch_arguments={"headless": headless, "battery": battery}.items(),
    )

    bt_runner = Node(
        package="spacetry_bt",
        executable="spacetry_bt_runner",
        name="spacetry_bt_runner",
        output="screen",
        parameters=[
            os.path.join(bt_share, "bt_params.yaml"),
            {
                "tree_file": os.path.join(bt_share, "trees", "base_bt.xml"),
                "use_sim_time": True,
            },
        ],
    )

    mission_server = Node(
        package="spacetry_mission",
        executable="spacetry_mission_server.py",
        name="spacetry_mission_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    scenario_driver = Node(
        package="spacetry_scenario_driver",
        executable="scenario_driver",
        name="spacetry_scenario_driver",
        output="screen",
        parameters=[
            {
                "scenario_config": scenario_config,
                "report_path": report_path,
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="0"),
            DeclareLaunchArgument("battery", default_value="1.0"),
            DeclareLaunchArgument(
                "report_path",
                default_value=os.path.join("/tmp", "spacetry_autonomy_report.json"),
            ),
            DeclareLaunchArgument(
                "scenario_config",
                default_value=os.path.join(scenario_share, "config", "autonomy_scenario.yaml"),
            ),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            SetParameter(name="use_sim_time", value=True),
            bringup_launch,
            mission_server,
            bt_runner,
            scenario_driver,
        ]
    )
