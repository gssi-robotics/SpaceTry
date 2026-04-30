#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-spacetry:dev}"
PLATFORM="${PLATFORM:-linux/amd64}"

docker buildx build \
  --platform "${PLATFORM}" \
  --load \
  --ssh default \
  -f docker/Dockerfile \
  -t "${IMAGE_NAME}" \
  "${ROOT_DIR}"

echo "Built ${IMAGE_NAME} (${PLATFORM})"
