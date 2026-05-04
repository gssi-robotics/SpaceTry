#!/usr/bin/env bash
set -euo pipefail

demo_root="${1:-/ws/src/space_ros_demos}"
curiosity_root="${demo_root}/curiosity_rover/curiosity_description/models/urdf"
sensor_mast="${curiosity_root}/sensor_mast.xacro"
gazebo_file="${curiosity_root}/curiosity_mars_rover.gazebo"

if [[ ! -f "${sensor_mast}" ]]; then
  echo "Expected Curiosity sensor mast file not found: ${sensor_mast}" >&2
  exit 1
fi

if [[ ! -f "${gazebo_file}" ]]; then
  echo "Expected Curiosity Gazebo file not found: ${gazebo_file}" >&2
  exit 1
fi

# The upstream file references a named URDF material without defining it,
# which triggers repeated parser warnings at runtime.
if grep -q '<material name="black"/>' "${sensor_mast}"; then
  perl -0pi -e 's{<material name="black"\s*/>}{<material name="black">\n\t\t\t\t\t<color rgba="0 0 0 1"/>\n\t\t\t\t</material>}g' "${sensor_mast}"
fi

grep -q '<color rgba="0 0 0 1"/>' "${sensor_mast}"

# Keep the Gazebo-specific frame tag for now. Removing it suppresses the
# SDFormat warning, but also changes the bridged LaserScan frame_id away
# from lidar_link in this stack.
grep -q '<gz_frame_id>lidar_link</gz_frame_id>' "${gazebo_file}"
