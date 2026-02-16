#!/usr/bin/env bash
set -e

# Download Gazebo Fuel models required for mars_outpost.sdf
# Downloads the complete model packages with all assets (meshes, textures, etc.)

FUEL_CACHE="${HOME}/.gz/fuel/fuel.gazebosim.org"
mkdir -p "$FUEL_CACHE"

echo "Downloading Gazebo Fuel models with all assets..."

# Function to download model package
download_model() {
  local model_url="$1"
  local owner=$(echo "$model_url" | awk -F'/' '{print $(NF-2)}')
  local model_name=$(echo "$model_url" | awk -F'/' '{print $NF}')
  local cache_path="${FUEL_CACHE}/${owner,,}/models/${model_name}"
  
  # Check if model already cached
  if [ -d "$cache_path" ] && [ -f "$cache_path/model.config" ]; then
    echo "✓ Already cached: $model_name"
    return 0
  fi
  
  echo "⬇ Downloading: $model_name"
  
  # Download using wget to get the full compressed package
  mkdir -p "${cache_path%/*}"
  
  # Try downloading the model.tar.gz
  if wget -q -O "${cache_path}.tar.gz" "${model_url}/1/files/model.tar.gz" 2>/dev/null; then
    tar -xzf "${cache_path}.tar.gz" -C "${cache_path%/*}/" 2>/dev/null && {
      rm -f "${cache_path}.tar.gz"
      echo "  ✓ Success"
      return 0
    }
  fi
  
  # Fallback: use gz fuel download
  if gz fuel download --url "$model_url" 2>/dev/null; then
    echo "  ✓ Success (metadata only)"
    return 0
  fi
  
  echo "  ⚠ Warning: Could not download $model_name"
  return 1
}

# Function to fix ISS model which has absolute HTTP URLs in its SDF
fix_iss_model() {
  local iss_path="${FUEL_CACHE}/openrobotics/models/international space station (half)/3"
  
  if [ ! -f "$iss_path/model.sdf" ]; then
    return 0
  fi
  
  echo "⬇ Downloading ISS mesh files..."
  
  # Create meshes directory
  mkdir -p "$iss_path/meshes"
  
  # Download the mesh file directly
  if wget -q -O "$iss_path/meshes/US_Lab_ISS_half.dae" \
    "https://fuel.gazebosim.org/1.0/openrobotics/models/international space station (half)/3/files/meshes/US_Lab_ISS_half.dae" 2>/dev/null; then
    echo "  ✓ ISS mesh downloaded"
  else
    echo "  ⚠ Warning: Could not download ISS mesh"
    return 0
  fi
  
  # Update model.sdf to use local mesh path instead of absolute URL
  if [ -f "$iss_path/model.sdf" ]; then
    sed -i 's|https://fuel.gazebosim.org/1.0/openrobotics/models/international space station (half)/3/files/meshes/|meshes/|g' "$iss_path/model.sdf"
    echo "  ✓ Updated ISS model.sdf to use local meshes"
  fi
}

# Array of models to download
declare -a models=(
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/International Space Station (half)"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Radio tower"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Medium Rock Fall 3"
  "https://fuel.gazebosim.org/1.0/Cole/models/Ocean Rock 07"
  "https://fuel.gazebosim.org/1.0/Cole/models/Ocean Rock 01"
)

for model_url in "${models[@]}"; do
  download_model "$model_url"
done

# Post-process ISS model to download and fix mesh references
fix_iss_model

echo "Fuel models download complete!"
