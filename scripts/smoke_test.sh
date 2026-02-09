#!/usr/bin/env bash
set -o pipefail
source /opt/ros/spaceros/setup.bash
source /ws/install/setup.bash

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

CONFIG_DIR="$(ros2 pkg prefix marti_mission)/share/marti_mission/config"

export MARTI_WORLD_SDF="${WORLD_FILE}"
export MARTI_MISSION_CONFIG_DIR="${CONFIG_DIR}"

python3 /ws/scripts/validate_mission_config.py
echo "OK: mission config validated"