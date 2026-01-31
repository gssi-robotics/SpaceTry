#!/usr/bin/env bash
# --- MARTI world smoke test: mars_outpost loads headless ---
set -e
# Avoid nounset issues while sourcing setup scripts
source /opt/ros/spaceros/setup.bash
source /ws/install/setup.bash
set -u
set -o pipefail

MODELS_ROOT="$(ros2 pkg prefix marti_models)/share/marti_models/models"
WORLD_FILE="$(ros2 pkg prefix marti_world)/share/marti_world/worlds/mars_outpost.sdf"

export GZ_SIM_RESOURCE_PATH="${MODELS_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"

# Run headless for a few seconds and capture logs
timeout 10s gz sim -s -r "${WORLD_FILE}" > /tmp/marti_world.log 2>&1 || true

# Fail on missing resources
if grep -Ei "Unable to find uri\[model://|Could not find|Error.*URI|No such file" /tmp/marti_world.log; then
  echo "Gazebo reported missing resources:"
  tail -200 /tmp/marti_world.log
  exit 1
fi

echo "OK: mars_outpost loaded headless"
