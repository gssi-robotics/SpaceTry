#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
COMPOSE_FILE="${ROOT_DIR}/docker/docker-compose.yaml"
SERVICE="spacetry"
CONTAINER_NAME=""
IMAGE="spacetry:dev"
BASE_IMAGE="docker.io/osrf/space-ros:jazzy-2025.10.0"
SKILL_PATH="${SKILL_DIR}"
SCENARIO_PACKAGE=""
RUN_CLASS="full_run"
REQUIRED_SKILL_COMMIT=""
REQUIRE_MAIN_RUN_READY=0
SKIP_DOCKER_AUTH_CHECK=0

CHECK_NAMES=()
CHECK_STATUSES=()
CHECK_MESSAGES=()

IMAGE_PRESENT_STATUS="FAIL"
CONTAINER_RUNNING_STATUS="FAIL"
DOCKER_AUTH_STATUS="SKIP"
IMAGE_FRESHNESS_STATUS="WARN"
SKILL_COMMIT_STATUS="WARN"
PACKAGE_SYNC_STATUS="WARN"

usage() {
  cat <<'EOF'
Usage: skills/spacetry-autonomy-scenario-driver/scripts/scenario_preflight.sh [options]

Checks whether the local Docker and repository state are ready for a
scenario-driver run, and whether the run is ready to serve as the main trusted
`full_run` for the current scenario iteration.

Options:
  --scenario-package <name>        ROS 2 scenario package under src/
  --skill-path <path>              Skill directory to validate
  --required-skill-commit <sha>    Commit hash required when main-run pinning matters
  --run-class <full_run|smoke|tuning>
                                   Run classification to evaluate
  --service <name>                 Docker Compose service name
  --container-name <name>          Explicit container name/id override
  --compose-file <path>            Compose file to use
  --image <name>                   Local image to inspect
  --base-image <name>              Registry image used for auth probe
  --skip-docker-auth-check         Skip remote registry auth validation
  --require-main-run-ready         Exit non-zero unless the run is ready to
                                   serve as the main trusted `full_run`
  --help                           Show this help
EOF
}

relative_to_root() {
  local path="$1"
  if [[ "$path" == "${ROOT_DIR}/"* ]]; then
    printf '%s\n' "${path#"${ROOT_DIR}/"}"
  else
    printf '%s\n' "$path"
  fi
}

record_check() {
  CHECK_NAMES+=("$1")
  CHECK_STATUSES+=("$2")
  CHECK_MESSAGES+=("$3")
}

container_ref() {
  if [[ -n "$CONTAINER_NAME" ]]; then
    printf '%s\n' "$CONTAINER_NAME"
    return 0
  fi
  docker compose -f "$COMPOSE_FILE" ps -q "$SERVICE" 2>/dev/null | head -n 1
}

scenario_dir_hash() {
  local base_dir="$1"
  local package_name="$2"
  tar \
    --sort=name \
    --mtime='UTC 1970-01-01' \
    --owner=0 \
    --group=0 \
    --numeric-owner \
    -cf - \
    -C "$base_dir" \
    "$package_name" \
    | sha256sum \
    | awk '{print $1}'
}

scenario_dir_hash_in_container() {
  local container="$1"
  local package_name="$2"
  docker exec "$container" bash -lc \
    "tar --sort=name --mtime='UTC 1970-01-01' --owner=0 --group=0 --numeric-owner -cf - -C /ws/src '$package_name'" \
    | sha256sum \
    | awk '{print $1}'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario-package)
      SCENARIO_PACKAGE="$2"
      shift 2
      ;;
    --skill-path)
      SKILL_PATH="$2"
      shift 2
      ;;
    --required-skill-commit)
      REQUIRED_SKILL_COMMIT="$2"
      shift 2
      ;;
    --run-class)
      RUN_CLASS="$2"
      shift 2
      ;;
    --service)
      SERVICE="$2"
      shift 2
      ;;
    --container-name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --compose-file)
      COMPOSE_FILE="$2"
      shift 2
      ;;
    --image)
      IMAGE="$2"
      shift 2
      ;;
    --base-image)
      BASE_IMAGE="$2"
      shift 2
      ;;
    --skip-docker-auth-check)
      SKIP_DOCKER_AUTH_CHECK=1
      shift
      ;;
    --require-main-run-ready)
      REQUIRE_MAIN_RUN_READY=1
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$RUN_CLASS" in
  full_run|smoke|tuning)
    ;;
  *)
    echo "Invalid --run-class: $RUN_CLASS" >&2
    exit 2
    ;;
esac

for required_cmd in docker git sha256sum tar date timeout; do
  if ! command -v "$required_cmd" >/dev/null 2>&1; then
    echo "Missing required command: $required_cmd" >&2
    exit 1
  fi
done

if [[ ! -f "$COMPOSE_FILE" ]]; then
  echo "Compose file not found: $COMPOSE_FILE" >&2
  exit 1
fi

if [[ ! -d "$SKILL_PATH" ]]; then
  echo "Skill path not found: $SKILL_PATH" >&2
  exit 1
fi

if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  IMAGE_PRESENT_STATUS="PASS"
  record_check "image_present" "PASS" "Local image '$IMAGE' exists."
else
  record_check "image_present" "FAIL" "Local image '$IMAGE' is missing."
fi

CONTAINER_REF="$(container_ref || true)"
if [[ -n "$CONTAINER_REF" ]] && docker inspect -f '{{.State.Running}}' "$CONTAINER_REF" 2>/dev/null | grep -qx 'true'; then
  CONTAINER_RUNNING_STATUS="PASS"
  record_check "container_running" "PASS" "Container '$CONTAINER_REF' is running."
else
  record_check "container_running" "FAIL" "No running container found for service '$SERVICE'."
fi

if (( SKIP_DOCKER_AUTH_CHECK )); then
  DOCKER_AUTH_STATUS="SKIP"
  record_check "docker_auth" "SKIP" "Skipped Docker registry auth probe for '$BASE_IMAGE'."
else
  if auth_output="$(timeout 30 docker buildx imagetools inspect "$BASE_IMAGE" 2>&1)"; then
    DOCKER_AUTH_STATUS="PASS"
    record_check "docker_auth" "PASS" "Registry metadata for '$BASE_IMAGE' is readable."
  else
    DOCKER_AUTH_STATUS="WARN"
    auth_output="${auth_output//$'\n'/ }"
    record_check "docker_auth" "WARN" "Registry metadata probe failed: ${auth_output}"
  fi
fi

if [[ "$IMAGE_PRESENT_STATUS" == "PASS" ]]; then
  image_created="$(docker image inspect --format '{{.Created}}' "$IMAGE")"
  image_created_epoch="$(date -d "$image_created" +%s)"
  latest_image_input_epoch="$(
    git -C "$ROOT_DIR" log -1 --format=%ct -- docker deps scripts/build.sh 2>/dev/null || true
  )"
  if [[ -n "$latest_image_input_epoch" ]] && (( image_created_epoch >= latest_image_input_epoch )); then
    IMAGE_FRESHNESS_STATUS="PASS"
    record_check "image_freshness" "PASS" "Image '$IMAGE' is newer than the latest committed Docker/deps change."
  elif [[ -n "$latest_image_input_epoch" ]]; then
    IMAGE_FRESHNESS_STATUS="WARN"
    record_check "image_freshness" "WARN" "Image '$IMAGE' is older than the latest committed Docker/deps change."
  else
    IMAGE_FRESHNESS_STATUS="WARN"
    record_check "image_freshness" "WARN" "Could not determine the latest committed Docker/deps change."
  fi
else
  record_check "image_freshness" "WARN" "Image freshness was not checked because '$IMAGE' is missing."
fi

skill_rel_path="$(relative_to_root "$SKILL_PATH")"
skill_commit="$(git -C "$ROOT_DIR" log -1 --format=%H -- "$skill_rel_path" 2>/dev/null || true)"
skill_dirty="$(git -C "$ROOT_DIR" status --porcelain -- "$skill_rel_path" 2>/dev/null || true)"
if [[ -z "$skill_commit" ]]; then
  SKILL_COMMIT_STATUS="FAIL"
  record_check "skill_commit" "FAIL" "Could not determine a git commit for '$skill_rel_path'."
elif [[ -n "$REQUIRED_SKILL_COMMIT" ]] && [[ "$skill_commit" != "$REQUIRED_SKILL_COMMIT" ]]; then
  SKILL_COMMIT_STATUS="WARN"
  record_check "skill_commit" "WARN" "Skill commit is '$skill_commit', expected '$REQUIRED_SKILL_COMMIT'."
elif [[ -n "$skill_dirty" ]]; then
  SKILL_COMMIT_STATUS="WARN"
  record_check "skill_commit" "WARN" "Skill tree is dirty at commit '$skill_commit'."
else
  SKILL_COMMIT_STATUS="PASS"
  record_check "skill_commit" "PASS" "Skill commit is '$skill_commit' and the tracked files are clean."
fi

if [[ -n "$SCENARIO_PACKAGE" ]]; then
  host_package_dir="${ROOT_DIR}/src/${SCENARIO_PACKAGE}"
  if [[ ! -d "$host_package_dir" ]]; then
    PACKAGE_SYNC_STATUS="FAIL"
    record_check "package_sync" "FAIL" "Host package '${SCENARIO_PACKAGE}' does not exist under src/."
  elif [[ "$CONTAINER_RUNNING_STATUS" != "PASS" ]]; then
    PACKAGE_SYNC_STATUS="FAIL"
    record_check "package_sync" "FAIL" "Cannot verify /ws/src sync because the container is not running."
  elif ! docker exec "$CONTAINER_REF" bash -lc "test -d /ws/src/'$SCENARIO_PACKAGE'" >/dev/null 2>&1; then
    PACKAGE_SYNC_STATUS="WARN"
    record_check "package_sync" "WARN" "Package '${SCENARIO_PACKAGE}' is not present inside /ws/src."
  else
    host_hash="$(scenario_dir_hash "${ROOT_DIR}/src" "$SCENARIO_PACKAGE")"
    container_hash="$(scenario_dir_hash_in_container "$CONTAINER_REF" "$SCENARIO_PACKAGE")"
    if [[ "$host_hash" == "$container_hash" ]]; then
      PACKAGE_SYNC_STATUS="PASS"
      record_check "package_sync" "PASS" "Host and container copies of '${SCENARIO_PACKAGE}' match."
    else
      PACKAGE_SYNC_STATUS="WARN"
      record_check "package_sync" "WARN" "Host and container copies of '${SCENARIO_PACKAGE}' differ."
    fi
  fi
else
  record_check "package_sync" "WARN" "No scenario package was provided, so /ws/src sync was not checked."
fi

main_run_ready=1
main_run_reasons=()

if [[ "$RUN_CLASS" != "full_run" ]]; then
  main_run_ready=0
  main_run_reasons+=("run_class=${RUN_CLASS} is not eligible to serve as the primary full_run result")
fi

if [[ "$IMAGE_PRESENT_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("local image '${IMAGE}' is missing")
fi

if [[ "$IMAGE_FRESHNESS_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("image freshness check did not pass")
fi

if [[ -n "$REQUIRED_SKILL_COMMIT" ]] && [[ "$SKILL_COMMIT_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("required skill commit pinning did not pass")
fi

if [[ "$CONTAINER_RUNNING_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("scenario container is not running")
fi

if [[ -z "$SCENARIO_PACKAGE" ]]; then
  main_run_ready=0
  main_run_reasons+=("scenario package was not provided, so sync eligibility could not be checked")
elif [[ "$PACKAGE_SYNC_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("host-vs-container scenario package sync did not pass")
fi

printf 'Scenario preflight summary\n'
printf '  repo_root: %s\n' "$ROOT_DIR"
printf '  run_class: %s\n' "$RUN_CLASS"
printf '  image: %s\n' "$IMAGE"
printf '  service: %s\n' "$SERVICE"
if [[ -n "$SCENARIO_PACKAGE" ]]; then
  printf '  scenario_package: %s\n' "$SCENARIO_PACKAGE"
fi
printf '\nChecks\n'
for idx in "${!CHECK_NAMES[@]}"; do
  printf '  - %-16s %-5s %s\n' "${CHECK_NAMES[$idx]}" "${CHECK_STATUSES[$idx]}" "${CHECK_MESSAGES[$idx]}"
done

printf '\nMain full_run readiness\n'
if (( main_run_ready )); then
  printf '  eligible: yes\n'
else
  printf '  eligible: no\n'
  for reason in "${main_run_reasons[@]}"; do
    printf '  - %s\n' "$reason"
  done
fi

basic_failures=0
for status in "${CHECK_STATUSES[@]}"; do
  if [[ "$status" == "FAIL" ]]; then
    basic_failures=$((basic_failures + 1))
  fi
done

if (( basic_failures > 0 )); then
  exit 1
fi

if (( REQUIRE_MAIN_RUN_READY )) && (( ! main_run_ready )); then
  exit 1
fi
