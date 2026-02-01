#!/usr/bin/env bash
set -e

# If /ws is bind-mounted, /ws/src may not contain imported deps.
# Provide deps via a symlink to the image cache (offline, pinned).
if [ ! -e /ws/src/space_ros_demos ] && [ -d /opt/marti_deps/space_ros_demos ]; then
  mkdir -p /ws/src
  echo "[entrypoint] Linking /ws/src/space_ros_demos -> /opt/marti_deps/space_ros_demos"
  ln -s /opt/marti_deps/space_ros_demos /ws/src/space_ros_demos
fi

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
