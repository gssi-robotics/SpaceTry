#!/usr/bin/env bash
set -e

# Load global environment (/etc/profile.d/spacetry_ros_env.sh + spacetry_deps_link.sh)
if [ -f /etc/profile ]; then
  # shellcheck disable=SC1091
  source /etc/profile
fi

# Source merged workspace overlay if present
if [ -f "/ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

exec "$@"
