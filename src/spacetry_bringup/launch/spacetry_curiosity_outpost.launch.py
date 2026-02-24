"""
SpaceTry 🥐 bringup: Curiosity rover in the SpaceTry 🥐 mars_outpost world.

- Read spawn pose from spacetry_mission/config/waypoints.yaml (dock_pad_01)
- Launch mars_outpost
- Spawn Curiosity via ros_gz_sim create (-param robot_description)
- Start bridges + controllers
- Include the demo nodes (mars_rover.launch.py)
"""

import os
import math
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _load_waypoint_pose(waypoints_yaml: str, waypoint_name: str):
    """
    Returns (x, y, yaw) from the YAML for the given waypoint.
    If missing, falls back to (0,0,0) with a clear console message.
    """
    try:
        with open(waypoints_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        wps = (data.get("waypoints") or {})
        wp = wps.get(waypoint_name) or {}
        x = float(wp.get("x", 0.0))
        y = float(wp.get("y", 0.0))
        yaw = float(wp.get("yaw", 0.0))
        return (x, y, yaw)
    except Exception as e:
        print(f"[spacetry_bringup] WARN: failed to read {waypoints_yaml}: {e}")
        return (0.0, 0.0, 0.0)


def generate_launch_description():
    # --- launch args
    headless = LaunchConfiguration("headless")
    spawn_waypoint = LaunchConfiguration("spawn_waypoint")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_x_offset = LaunchConfiguration("spawn_x_offset")
    spawn_y_offset = LaunchConfiguration("spawn_y_offset")
    spawn_yaw_offset = LaunchConfiguration("spawn_yaw_offset")
    battery = LaunchConfiguration("battery")

    # --- paths
    spacetry_world_share = get_package_share_directory("spacetry_world")
    spacetry_models_root = os.path.join(get_package_share_directory("spacetry_models"), "models")
    spacetry_mission_cfg = os.path.join(get_package_share_directory("spacetry_mission"), "config", "waypoints.yaml")

    curiosity_gazebo_share = get_package_share_directory("curiosity_gazebo")
    curiosity_desc_share = get_package_share_directory("curiosity_description")

    # Curiosity URDF (xacro)
    urdf_xacro = os.path.join(curiosity_desc_share, "models", "urdf", "curiosity_mars_rover.xacro")

    # Build robot_description XML
    doc = xacro.process_file(urdf_xacro)
    robot_xml = doc.toxml()
    robot_description_param = {"robot_description": robot_xml}

    # --- env: make sure Gazebo can find BOTH SpaceTry 🥐 models + Curiosity models/meshes
    env_gz_plugin = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                os.environ.get("LD_LIBRARY_PATH", ""),
            ]
        ),
    )
    env_gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                spacetry_models_root,
                os.path.join(curiosity_gazebo_share, "models"),
                curiosity_desc_share,
            ]
        ),
    )

    # --- launch SpaceTry 🥐 world (your existing launch)
    mars_outpost_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spacetry_world_share, "launch", "mars_outpost.launch.py")),
        launch_arguments={"headless": headless}.items(),
    )

    # --- robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # --- spawn pose (Approach 1: YAML waypoint)
    # Note: LaunchConfiguration values are strings; we read the YAML here using the default waypoint name,
    # but we also allow offsets via launch args for “spawn close to dock_pad_01”.
    waypoint_name = "dock_pad_01"  # default
    try:
        # If user passes spawn_waypoint at runtime, it will override, but we keep the default stable.
        waypoint_name = os.environ.get("SPACETRY_SPAWN_WAYPOINT", waypoint_name)
    except Exception:
        pass

    base_x, base_y, base_yaw = _load_waypoint_pose(spacetry_mission_cfg, waypoint_name)

    # Spawn Curiosity via ros_gz_sim create using -param robot_description
    # ros_gz_sim/create.cpp shows -param reads XML from a ROS param on the create node, and -Y is yaw in radians. :contentReference[oaicite:0]{index=0}
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_curiosity",
        output="screen",
        parameters=[robot_description_param],
        arguments=[
            "-name", "curiosity_mars_rover",
            "-param", "robot_description",
            "-x", str(base_x),
            "-y", str(base_y),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", str(base_yaw),
        ],
    )

    # --- Curiosity helpers / bridges (matching upstream demo intent)
    odom_node = Node(
        package="curiosity_gazebo",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/curiosity_mars_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/image_raw", "/image_raw"],
        output="screen",
    )

    battery_params = PathJoinSubstitution(
        [FindPackageShare("spacetry_battery"), "config", "battery_manager.yaml"]
    )

    battery_manager = Node(
        package="spacetry_battery",
        executable="battery_manager",
        name="battery_manager",
        output="screen",
        parameters=[
            battery_params,
            {
                "outpost_x": base_x,
                "outpost_y": base_y,
                "initial_soc": ParameterValue(battery, value_type=float),
            },
        ],
    )

    # --- controllers: same sequence used by the Space ROS demo
    component_state_msg = '{name: "GazeboSystem", target_state: {id: 3, label: ""}}'

    set_hardware_interface_active = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "controller_manager/set_hardware_component_state",
            "controller_manager_msgs/srv/SetHardwareComponentState",
            component_state_msg,
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "arm_joint_trajectory_controller"],
        output="screen",
    )
    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "mast_joint_trajectory_controller"],
        output="screen",
    )
    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "wheel_velocity_controller"],
        output="screen",
    )
    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "steer_position_controller"],
        output="screen",
    )
    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "wheel_tree_position_controller"],
        output="screen",
    )

    # --- include demo nodes (arm/mast/wheel/run_demo)
    mars_rover_demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("curiosity_rover_demo"), "launch", "mars_rover.launch.py")
        )
    )


    # --- LiDAR obstacle direction (front/left/right topics for BT)
    obstacle_direction = Node(
        package="spacetry_perception",
        executable="obstacle_direction_node",
        name="obstacle_direction",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scan_topic": "/scan",
            "base_frame": "base_link",
            "threshold_m": 10.0,
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="1",
                description="Run gz sim headless (server-only) if supported by the world launch.",
            ),
            DeclareLaunchArgument(
                "spawn_waypoint",
                default_value="dock_pad_01",
                description="Waypoint name in spacetry_mission/config/waypoints.yaml to spawn near.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.30",
                description="Spawn Z (meters).",
            ),
            DeclareLaunchArgument(
                "spawn_x_offset",
                default_value="0.0",
                description="(Reserved) Spawn X offset from the waypoint.",
            ),
            DeclareLaunchArgument(
                "spawn_y_offset",
                default_value="0.0",
                description="(Reserved) Spawn Y offset from the waypoint.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw_offset",
                default_value="0.0",
                description="(Reserved) Spawn yaw offset (radians) from the waypoint.",
            ),
            DeclareLaunchArgument(
                "battery",
                default_value="1.0",
                description="Initial battery SOC fraction (0.0..1.0). Example: battery:=0.2 starts at 20%.",
            ),
            env_gz_plugin,
            env_gz_resource,
            SetParameter(name="use_sim_time", value=True),
            mars_outpost_launch,
            robot_state_publisher,
            spawn,
            odom_node,
            ros_gz_bridge,
            image_bridge,
            battery_manager,
            obstacle_direction,
            # Controller chain
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[set_hardware_interface_active])),
            RegisterEventHandler(OnProcessExit(target_action=set_hardware_interface_active, on_exit=[load_joint_state_broadcaster])),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[
                        load_arm_joint_traj_controller,
                        load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller,
                    ],
                )
            ),
            mars_rover_demo_nodes,
        ]
    )
