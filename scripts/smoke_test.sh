#!/usr/bin/env bash
set -e
# IMPORTANT: don't enable nounset until after sourcing setup scripts
source /opt/ros/spaceros/setup.bash

# If the workspace isn't built (common in CI when mounting into /ws), build it now
if [ ! -f /ws/install/setup.bash ]; then
  echo "[smoke_test] /ws/install/setup.bash missing; building workspace..."
  cd /ws
  colcon build --event-handlers console_direct+
fi

source /ws/install/setup.bash

set -u
set -o pipefail

MODELS_ROOT="$(ros2 pkg prefix marti_models)/share/marti_models/models"
WORLD_FILE="$(ros2 pkg prefix marti_world)/share/marti_world/worlds/mars_outpost.sdf"
export GZ_SIM_RESOURCE_PATH="${MODELS_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"

timeout 10s gz sim -s -r "${WORLD_FILE}" > /tmp/marti_world.log 2>&1 || true

if grep -Ei "Unable to find uri\[model://|Could not find|Error.*URI|No such file" /tmp/marti_world.log; then
  echo "Gazebo reported missing resources:"
  tail -200 /tmp/marti_world.log
  exit 1
fi

echo "OK: mars_outpost loaded headless"