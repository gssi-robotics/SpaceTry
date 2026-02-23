#!/usr/bin/env bash
set -euo pipefail

# Auto-detect host's render group GID for GPU device access inside the container
export RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3 || echo 109)

docker compose -f docker/docker-compose.yaml up --build -d
