#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-marti:dev}"

echo "Running smoke test for ${IMAGE_NAME}"
docker run --rm "${IMAGE_NAME}" bash -lc 'echo "Container OK"; uname -a'
