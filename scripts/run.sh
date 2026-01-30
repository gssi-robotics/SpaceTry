#!/usr/bin/env bash
set -euo pipefail
docker compose -f docker/docker-compose.yaml up --build
