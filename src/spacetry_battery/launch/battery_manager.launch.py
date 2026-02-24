from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("spacetry_battery"), "config", "battery_manager.yaml"])
    return LaunchDescription([
        Node(
            package="spacetry_battery",
            executable="battery_manager",
            name="battery_manager",
            output="screen",
            parameters=[params],
        )
    ])
