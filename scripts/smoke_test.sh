#!/usr/bin/env bash
set -euo pipefail

WORLD="/ws/src/marti_worlds/worlds/empty.sdf"

echo "== Checking tools =="
command -v gz
gz sim --versions
command -v ros2
ros2 -h >/dev/null

echo "== Starting headless Gazebo =="
gz sim -s -r "${WORLD}" > /tmp/gz_smoke.log 2>&1 &
GZ_PID=$!

sleep 3

gz topic -l >/tmp/gz_topics.txt 2>&1 || true
test -s /tmp/gz_topics.txt

echo "== Stopping Gazebo =="
kill "${GZ_PID}"
wait "${GZ_PID}" || true

echo "== Smoke test OK =="
tail -n 50 /tmp/gz_smoke.log || true
