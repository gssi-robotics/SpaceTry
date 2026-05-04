#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${ROOT_DIR}/log"
COMPOSE_FILE="${ROOT_DIR}/docker/docker-compose.yaml"
SERVICE="spacetry"

running_container_ref() {
  local ref=""
  for _ in 1 2 3 4 5; do
    ref="$(docker compose -f "${COMPOSE_FILE}" ps -q "${SERVICE}" | head -n 1)"
    if [[ -n "${ref}" ]] && docker inspect --format '{{.State.Running}}' "${ref}" 2>/dev/null | grep -qx 'true'; then
      printf '%s\n' "${ref}"
      return 0
    fi
    sleep 1
  done
  return 1
}

if [ -e "${LOG_DIR}" ] && [ ! -d "${LOG_DIR}" ]; then
  printf 'Error: %s exists but is not a directory.\n' "${LOG_DIR}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

if ! write_test="$(mktemp "${LOG_DIR}/.spacetry-write-test.XXXXXX" 2>/dev/null)"; then
  printf 'Error: %s is not writable by the current user.\n' "${LOG_DIR}" >&2
  printf 'Fix the host directory permissions before starting the container.\n' >&2
  exit 1
fi

rm -f "${write_test}"

DOCKER_BUILDKIT=1 docker compose -f "${COMPOSE_FILE}" up -d "${SERVICE}"

container_ref="$(running_container_ref || true)"
if [[ -z "${container_ref}" ]]; then
  printf "Error: service '%s' did not reach a running state.\n" "${SERVICE}" >&2
  exit 1
fi
