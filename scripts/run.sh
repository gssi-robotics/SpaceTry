#!/usr/bin/env bash
set -euo pipefail
DOCKER_BUILDKIT=1 docker compose -f docker/docker-compose.yaml up --build -d
