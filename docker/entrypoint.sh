#!/usr/bin/env bash
set -e

# Provide deps via a symlink to the image cache (offline, pinned).
if [ -d /opt/marti_deps/space_ros_demos ]; then
  mkdir -p /ws/src
  if [ -L /ws/src/space_ros_demos ]; then
    # already linked
    :
  elif [ -e /ws/src/space_ros_demos ]; then
    echo "[entrypoint] Removing existing /ws/src/space_ros_demos (replacing with cached symlink)"
    rm -rf /ws/src/space_ros_demos
    ln -s /opt/marti_deps/space_ros_demos /ws/src/space_ros_demos
  else
    echo "[entrypoint] Linking /ws/src/space_ros_demos -> /opt/marti_deps/space_ros_demos"
    ln -s /opt/marti_deps/space_ros_demos /ws/src/space_ros_demos
  fi
fi

# Load global environment (includes /etc/profile.d/marti_ros_env.sh)
if [ -f /etc/profile ]; then
  source /etc/profile
fi


exec "$@"
