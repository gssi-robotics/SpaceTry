#!/usr/bin/env bash
set -euo pipefail
IMAGE_NAME="${IMAGE_NAME:-marti:dev}"
docker build -f docker/Dockerfile -t "${IMAGE_NAME}" .
echo "Built ${IMAGE_NAME}"
