#!/usr/bin/env bash
set -e

# Provide deps via a symlink to the image cache (offline, pinned).
# Always force the symlink so the workspace never sees a stale directory copy.
if [ -d /opt/marti_deps/space_ros_demos ]; then
  mkdir -p /ws/src
  rm -rf /ws/src/space_ros_demos
  ln -s /opt/marti_deps/space_ros_demos /ws/src/space_ros_demos
  echo "[entrypoint] Forced /ws/src/space_ros_demos -> /opt/marti_deps/space_ros_demos"
fi

# Load global environment (includes /etc/profile.d/marti_ros_env.sh)
# (this should set up /opt/ros/jazzy and /opt/ros/spaceros)
if [ -f /etc/profile ]; then
  # shellcheck disable=SC1091
  source /etc/profile
fi

# Source workspace overlay (preferred: merged install)
if [ -f "/ws/install/setup.bash" ]; then
  echo "[entrypoint] Sourcing workspace overlay: /ws/install/setup.bash"
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
else
  # Fallback for isolated installs (legacy): source each prefix if present
  if [ -d "/ws/install" ]; then
    echo "[entrypoint] No merged setup.bash found; sourcing isolated prefixes under /ws/install/*"
    for p in /ws/install/*; do
      if [ -f "$p/local_setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$p/local_setup.bash"
      fi
    done
  fi
fi

exec "$@"
