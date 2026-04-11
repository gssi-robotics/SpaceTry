#!/bin/bash
# verify_world.sh - Validate SpaceTry World SDF and Model URIs
#
# This script validates:
# 1. SDF syntax is correct
# 2. All model URIs in the world can be resolved
#
# Usage: ./scripts/verify_world.sh
# or:    docker compose exec spacetry /ws/scripts/verify_world.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"

SDF_FILE="$WORKSPACE_ROOT/src/spacetry_world/worlds/mars_outpost.sdf"
MODEL_DIR="$WORKSPACE_ROOT/src/spacetry_models/models"
DEMOS_COMMIT="acf369b"

echo "=========================================="
echo "SpaceTry World Verification Script"
echo "=========================================="
echo ""
echo "Workspace: $WORKSPACE_ROOT"
echo "SDF File:  $SDF_FILE"
echo "Models:    $MODEL_DIR"
echo ""

# Check prerequisites
if [ ! -f "$SDF_FILE" ]; then
  echo "ERROR: SDF file not found: $SDF_FILE"
  exit 1
fi

if [ ! -d "$MODEL_DIR" ]; then
  echo "ERROR: Model directory not found: $MODEL_DIR"
  exit 1
fi

# Install Gazebo if not available
if ! command -v gz &> /dev/null; then
  echo "[1/4] Installing Gazebo Harmonic..."
  apt-get update -qq
  apt-get install -y -qq wget lsb-release gnupg curl ca-certificates
  wget -q https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
  apt-get update -qq
  apt-get install -y -qq gz-harmonic
  echo "✓ Gazebo installed"
else
  echo "[1/4] Gazebo already installed: $(gz sim --version 2>/dev/null || echo 'harmonic')"
fi

# Fetch external models
echo "[2/4] Fetching external models from space-ros/demos..."
mkdir -p /tmp/spacetry_verify_{src,models}
curl -sL "https://github.com/space-ros/demos/archive/${DEMOS_COMMIT}.tar.gz" \
  | tar xz -C /tmp/spacetry_verify_src --strip-components=1 2>/dev/null
if [ -d "/tmp/spacetry_verify_src/curiosity_rover/curiosity_gazebo/models" ]; then
  cp -a /tmp/spacetry_verify_src/curiosity_rover/curiosity_gazebo/models/* /tmp/spacetry_verify_models/
  echo "✓ External models fetched: $(ls /tmp/spacetry_verify_models/ | wc -l) models"
else
  echo "⚠ External models not found in archive, continuing with local models only"
fi

# Validate SDF syntax
echo "[3/4] Validating SDF syntax..."
export GZ_SIM_RESOURCE_PATH="$MODEL_DIR:/tmp/spacetry_verify_models"
export SDF_PATH="$MODEL_DIR:/tmp/spacetry_verify_models"

if gz sdf -k "$SDF_FILE" &>/dev/null; then
  echo "✓ SDF syntax valid"
else
  echo "✗ SDF syntax invalid"
  echo "  Run: gz sdf -k $SDF_FILE"
  exit 1
fi

# Check model URIs
echo "[4/4] Checking model URIs..."
MODELS=$(grep -oP 'model://\K[^</"]+' "$SDF_FILE" | sort -u || true)

if [ -z "$MODELS" ]; then
  echo "⚠ No models found in SDF (or grep failed)"
else
  MISSING=0
  MODEL_COUNT=0
  
  for model in $MODELS; do
    MODEL_COUNT=$((MODEL_COUNT + 1))
    if [ -d "$MODEL_DIR/$model" ]; then
      echo "  ✓ $model (repo)"
    elif [ -d "/tmp/spacetry_verify_models/$model" ]; then
      echo "  ✓ $model (external)"
    else
      echo "  ✗ MISSING: $model"
      MISSING=1
    fi
  done
  
  if [ "$MISSING" -ne 0 ]; then
    echo ""
    echo "ERROR: One or more model URIs could not be resolved"
    exit 1
  fi
  echo "✓ All $MODEL_COUNT model URIs resolved"
fi

# Cleanup
rm -rf /tmp/spacetry_verify_src /tmp/spacetry_verify_models

echo ""
echo "=========================================="
echo "✓ All verification checks PASSED"
echo "=========================================="
echo ""
echo "The world is ready for use. You can now:"
echo "  • Launch: ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py"
echo "  • Run scenarios with confidence that the world is valid"
echo ""
