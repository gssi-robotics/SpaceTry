import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # --- Paths (all resolved via package shares) ---
    marti_world_share = get_package_share_directory("marti_world")
    marti_models_share = get_package_share_directory("marti_models")

    curiosity_gazebo_share = get_package_share_directory("curiosity_gazebo")
    curiosity_desc_share = get_package_share_directory("curiosity_description")
    curiosity_demo_share = get_package_share_directory("curiosity_rover_demo")

    # MARTI world launch
    marti_world_launch = os.path.join(marti_world_share, "launch", "mars_outpost.launch.py")

    # Curiosity model (xacro)
    urdf_xacro = os.path.join(curiosity_desc_share, "models", "urdf", "curiosity_mars_rover.xacro")

    # --- Environment: Gazebo plugin/resource paths ---
    # Match upstream pattern: include LD_LIBRARY_PATH in GZ_SIM_SYSTEM_PLUGIN_PATH
    env_gz_plugin = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                os.environ.get("LD_LIBRARY_PATH", ""),
            ]
        ),
    )

    # Resource path must include:
    # - MARTI models directory that contains model folders
    # - Curiosity gazebo models directory
    # - Curiosity description (meshes, etc.)
    marti_models_root = os.path.join(marti_models_share, "models")
    curiosity_models_root = os.path.join(curiosity_gazebo_share, "models")

    env_gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join(
            [
                marti_models_root,
                curiosity_models_root,
                curiosity_desc_share,
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ]
        ),
    )

    # --- Robot description (xacro -> robot_state_publisher) ---
    doc = xacro.process_file(urdf_xacro)
    robot_description = {"robot_description": doc.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # --- Start MARTI world (Gazebo) ---
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(marti_world_launch),
        # Keep your world launch default headless behavior (as-is).
        launch_arguments={}.items(),
    )

    # --- Spawn Curiosity into the running sim ---
    # Use the same name the upstream bridge expects.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "curiosity_mars_rover",
            "-topic",
            robot_description,
            # Positioning: keep upstream default z for now; we’ll refine after you choose pose policy.
            "-z",
            "-7.5",
        ],
        output="screen",
    )

    # --- Bridges (match upstream topics) ---
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

    odom_node = Node(
        package="curiosity_gazebo",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        output="screen",
    )

    # --- Controllers (match upstream chain) ---
    component_state_msg = '{name: "GazeboSystem", target_state: {id: 3, label: ""}}'

    set_hardware_interface_active = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "controller_manager/set_hardware_component_state",
            "controller_manager_msgs/srv/SetHardwareComponentState",
            component_state_msg,
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "mast_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_velocity_controller",
        ],
        output="screen",
    )

    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steer_position_controller",
        ],
        output="screen",
    )

    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_tree_position_controller",
        ],
        output="screen",
    )

    # Start demo control nodes (does not start Gazebo)
    demo_launch = os.path.join(curiosity_demo_share, "launch", "mars_rover.launch.py")
    start_demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={}.items(),
    )

    # Controller load sequence (same as upstream): after spawn -> activate hardware -> load controllers
    controller_chain = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
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
    ]

    # Small delay before spawn to let gz sim start reliably
    spawn_after_world = TimerAction(period=2.0, actions=[spawn])

    return LaunchDescription(
        [
            env_gz_plugin,
            env_gz_resource,
            SetParameter(name="use_sim_time", value=True),
            start_world,
            robot_state_publisher,
            ros_gz_bridge,
            image_bridge,
            odom_node,
            spawn_after_world,
            *controller_chain,
            start_demo_nodes,
        ]
    )
