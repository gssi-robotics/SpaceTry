#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-marti:dev}"
PLATFORM="${PLATFORM:-linux/amd64}"

docker buildx build --platform "${PLATFORM}" --load -f docker/Dockerfile -t "${IMAGE_NAME}" .

echo "Built ${IMAGE_NAME} (${PLATFORM})"
