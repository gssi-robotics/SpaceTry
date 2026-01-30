#!/usr/bin/env bash
set -e

# Source Space ROS environment (Space ROS installs under /opt/ros/spaceros)
if [ -f "/opt/ros/spaceros/setup.bash" ]; then
  source "/opt/ros/spaceros/setup.bash"
elif [ -f "/opt/ros/spaceros/setup.sh" ]; then
  source "/opt/ros/spaceros/setup.sh"
fi

# Source workspace overlay if present
if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

exec "$@"
