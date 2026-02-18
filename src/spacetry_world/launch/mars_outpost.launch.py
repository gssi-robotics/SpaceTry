from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    headless = LaunchConfiguration("headless")

    models_root = os.path.join(get_package_share_directory("marti_models"), "models")
    curiosity_models = os.path.join(get_package_share_directory("curiosity_gazebo"), "models")
    world_file = os.path.join(get_package_share_directory("marti_world"), "worlds", "mars_outpost.sdf")

    # Prepend our models paths to any existing resource path
    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{models_root}:{curiosity_models}" if gz_path == "" else f"{models_root}:{curiosity_models}:{gz_path}"

    gz_args = ["gz", "sim", "-r", world_file]
    # Headless server mode
    # -s starts server-only; still fine even if headless=0, but keep switch explicit
    gz_args_headless = ["gz", "sim", "-s", "-r", world_file]

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="1",
            description="Run gz sim in headless (-s) server mode (1) or normal mode (0)."
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_gz_path),
        ExecuteProcess(
            cmd=gz_args_headless,
            condition=IfCondition(headless),
            output="screen"
        ),
        ExecuteProcess(
            cmd=gz_args,
            condition=UnlessCondition(headless),
            output="screen"
        ),
    ])
