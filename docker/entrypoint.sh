#!/usr/bin/env bash
set -e

# Load global environment (/etc/profile.d/marti_ros_env.sh + marti_deps_link.sh)
if [ -f /etc/profile ]; then
  # shellcheck disable=SC1091
  source /etc/profile
fi

# Source merged workspace overlay if present
if [ -f "/ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

# Pre-download Gazebo Fuel models on first run
if [ -f "/ws/scripts/download_fuel_models.sh" ]; then
  bash /ws/scripts/download_fuel_models.sh
fi

exec "$@"
