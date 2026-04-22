from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    battery = LaunchConfiguration("battery")
    output_root = LaunchConfiguration("output_root")
    run_label = LaunchConfiguration("run_label")
    record_rosbag = LaunchConfiguration("record_rosbag")

    scenario_pkg = FindPackageShare("spacetry_scenario_navigation_obstacle_degraded_perception")
    bringup_pkg = FindPackageShare("spacetry_bringup")
    bt_pkg = FindPackageShare("spacetry_bt")

    scenario_contract = PathJoinSubstitution(
        [scenario_pkg, "config", "scenario_contract.yaml"]
    )
    scenario_config = PathJoinSubstitution(
        [scenario_pkg, "config", "scenario_config.yaml"]
    )
    baseline_tree_file = PathJoinSubstitution(
        [bt_pkg, "trees", "base_bt.xml"]
    )
    bringup_launch = PathJoinSubstitution(
        [bringup_pkg, "launch", "spacetry_curiosity_outpost.launch.py"]
    )

    baseline_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
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
                "use_sim_time": True,
                "scenario_contract_file": scenario_contract,
                "scenario_config_file": scenario_config,
                "output_root": output_root,
                "run_label": run_label,
                "record_rosbag": ParameterValue(record_rosbag, value_type=bool),
            }
        ],
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="1",
                description="Run Gazebo headless while the scenario driver executes.",
            ),
            DeclareLaunchArgument(
                "battery",
                default_value="1.0",
                description="Initial battery state of charge for the baseline bringup.",
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value="/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception",
                description="Host-visible directory used for reports, metrics, runtime logs, and rosbags.",
            ),
            DeclareLaunchArgument(
                "run_label",
                default_value="manual_run",
                description="Subdirectory label under output_root for this scenario execution.",
            ),
            DeclareLaunchArgument(
                "record_rosbag",
                default_value="true",
                description="Record a rosbag for the scenario run when true.",
            ),
            baseline_bringup,
            scenario_driver,
        ]
    )
