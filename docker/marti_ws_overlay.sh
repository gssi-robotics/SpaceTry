#!/usr/bin/env bash
# MARTI: container-wide environment hook (loaded by /etc/profile)

# Ensure Gazebo can find ROS/Gazebo system plugins (gz_ros2_control, etc.)
if [ -d /opt/ros/jazzy/lib ]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
fi
if [ -d /opt/ros/jazzy/lib/x86_64-linux-gnu ]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib/x86_64-linux-gnu:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
fi

# Auto-source workspace overlay when present (merged install)
if [ -f /ws/install/setup.bash ]; then
  . /ws/install/setup.bash
fi
