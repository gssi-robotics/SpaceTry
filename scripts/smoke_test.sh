#!/usr/bin/env bash
set -o pipefail
source /opt/ros/spaceros/setup.bash
colcon build --merge-install
source /ws/install/setup.bash

MODELS_ROOT="$(ros2 pkg prefix spacetry_models)/share/spacetry_models/models"
CURIOSITY_MODELS="$(ros2 pkg prefix curiosity_gazebo)/share/curiosity_gazebo/models"
WORLD_FILE="$(ros2 pkg prefix spacetry_world)/share/spacetry_world/worlds/mars_outpost.sdf"
export GZ_SIM_RESOURCE_PATH="${MODELS_ROOT}:${CURIOSITY_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"

timeout 10s gz sim -s -r "${WORLD_FILE}" > /tmp/spacetry_world.log 2>&1 || true

if grep -Ei "Unable to find uri\[model://|Could not find|Error.*URI|No such file" /tmp/spacetry_world.log; then
  echo "Gazebo reported missing resources:"
  tail -200 /tmp/spacetry_world.log
  exit 1
fi

echo "OK: mars_outpost loaded headless"

CONFIG_DIR="$(ros2 pkg prefix spacetry_mission)/share/spacetry_mission/config"

export SPACETRY_WORLD_SDF="${WORLD_FILE}"
export SPACETRY_MISSION_CONFIG_DIR="${CONFIG_DIR}"

python3 /ws/scripts/validate_mission_config.py
echo "OK: mission config validated"