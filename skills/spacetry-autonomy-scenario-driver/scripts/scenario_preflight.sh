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
RUNTIME_PACKAGES=()
RUN_CLASS="full_run"
REQUIRED_SKILL_CHECKSUM=""
REQUIRED_SKILL_COMMIT=""
REQUIRE_MAIN_RUN_READY=0
SKIP_DOCKER_AUTH_CHECK=0

CHECK_NAMES=()
CHECK_STATUSES=()
CHECK_MESSAGES=()

DOCKER_ACCESS_STATUS="FAIL"
IMAGE_PRESENT_STATUS="SKIP"
CONTAINER_RUNNING_STATUS="SKIP"
CONTAINER_IMAGE_STATUS="SKIP"
DOCKER_AUTH_STATUS="SKIP"
IMAGE_FRESHNESS_STATUS="SKIP"
SKILL_CHECKSUM_STATUS="FAIL"
SKILL_COMMIT_STATUS="WARN"
PACKAGE_SYNC_STATUS="SKIP"
PACKAGE_BUILD_STATUS="SKIP"

usage() {
  cat <<'EOF'
Usage: skills/spacetry-autonomy-scenario-driver/scripts/scenario_preflight.sh [options]

Checks whether the local Docker and repository state are ready for a
scenario-driver run, and whether the run is ready to serve as the main trusted
`full_run` for the current scenario iteration.

Options:
  --scenario-package <name>        Primary ROS 2 scenario package under src/
                                   (also included in runtime package checks)
  --runtime-package <name>         Additional repo-local runtime package under
                                   src/, repeat as needed
  --skill-path <path>              Skill directory to validate
  --required-skill-checksum <sha>  Skill tree checksum required when exact
                                   skill-state pinning matters
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

append_unique_runtime_package() {
  local package_name="$1"
  local existing_name
  for existing_name in "${RUNTIME_PACKAGES[@]}"; do
    if [[ "$existing_name" == "$package_name" ]]; then
      return 0
    fi
  done
  RUNTIME_PACKAGES+=("$package_name")
}

summarize_output() {
  local text="$1"
  text="${text//$'\n'/ }"
  printf '%s\n' "$text"
}

docker_daemon_probe() {
  timeout 30 docker version 2>&1
}

container_ref() {
  if [[ -n "$CONTAINER_NAME" ]]; then
    printf '%s\n' "$CONTAINER_NAME"
    return 0
  fi
  docker compose -f "$COMPOSE_FILE" ps -q "$SERVICE" | head -n 1
}

running_container_ref() {
  local attempt ref
  for attempt in 1 2 3 4 5; do
    ref="$(container_ref || true)"
    if [[ -n "$ref" ]] && docker inspect -f '{{.State.Running}}' "$ref" 2>/dev/null | grep -qx 'true'; then
      printf '%s\n' "$ref"
      return 0
    fi
    sleep 1
  done
  return 1
}

baseline_input_latest_epoch() {
  local search_roots=("${ROOT_DIR}/docker" "${ROOT_DIR}/deps")
  while IFS= read -r -d '' package_dir; do
    package_name="$(basename "$package_dir")"
    if [[ "$package_name" == spacetry_scenario_* ]]; then
      continue
    fi
    search_roots+=("$package_dir")
  done < <(
    find "${ROOT_DIR}/src" -mindepth 1 -maxdepth 1 -type d -print0 | LC_ALL=C sort -z
  )

  find "${search_roots[@]}" -type f -printf '%T@\n' | sort -n | tail -1
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

dir_tree_hash() {
  local dir_path="$1"
  local parent_dir
  local base_name

  parent_dir="$(cd "$(dirname "$dir_path")" && pwd)"
  base_name="$(basename "$dir_path")"

  tar \
    --sort=name \
    --mtime='UTC 1970-01-01' \
    --owner=0 \
    --group=0 \
    --numeric-owner \
    -cf - \
    -C "$parent_dir" \
    "$base_name" \
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

package_latest_mtime_in_container() {
  local container="$1"
  local package_name="$2"
  docker exec "$container" bash -lc \
    "find /ws/src/'$package_name' -type f -printf '%T@\n' | sort -n | tail -1"
}

package_install_marker_mtime_in_container() {
  local container="$1"
  local package_name="$2"
  docker exec "$container" bash -lc \
    "find /ws/install/share/'$package_name' -maxdepth 1 -type f \\( -name package.dsv -o -name package.xml \\) -printf '%T@\n' | sort -n | tail -1"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario-package)
      SCENARIO_PACKAGE="$2"
      shift 2
      ;;
    --runtime-package)
      append_unique_runtime_package "$2"
      shift 2
      ;;
    --skill-path)
      SKILL_PATH="$2"
      shift 2
      ;;
    --required-skill-checksum)
      REQUIRED_SKILL_CHECKSUM="$2"
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

if [[ -n "$SCENARIO_PACKAGE" ]]; then
  append_unique_runtime_package "$SCENARIO_PACKAGE"
fi

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

if docker_probe_output="$(docker_daemon_probe)"; then
  DOCKER_ACCESS_STATUS="PASS"
  record_check "docker_access" "PASS" "Docker daemon is reachable from this shell."
else
  docker_probe_output="$(summarize_output "$docker_probe_output")"
  record_check "docker_access" "FAIL" "Docker daemon probe failed: ${docker_probe_output}"
fi

CONTAINER_REF=""
if [[ "$DOCKER_ACCESS_STATUS" == "PASS" ]] && docker image inspect "$IMAGE" >/dev/null 2>&1; then
  IMAGE_PRESENT_STATUS="PASS"
  record_check "image_present" "PASS" "Local image '$IMAGE' exists."
elif [[ "$DOCKER_ACCESS_STATUS" == "PASS" ]]; then
  IMAGE_PRESENT_STATUS="FAIL"
  record_check "image_present" "FAIL" "Local image '$IMAGE' is missing."
else
  record_check "image_present" "SKIP" "Image presence was not checked because Docker daemon access failed."
fi

if [[ "$DOCKER_ACCESS_STATUS" == "PASS" ]]; then
  CONTAINER_REF="$(running_container_ref || true)"
fi
if [[ -n "$CONTAINER_REF" ]]; then
  CONTAINER_RUNNING_STATUS="PASS"
  record_check "container_running" "PASS" "Container '$CONTAINER_REF' is running."
elif [[ "$DOCKER_ACCESS_STATUS" == "PASS" ]]; then
  CONTAINER_RUNNING_STATUS="FAIL"
  record_check "container_running" "FAIL" "No running container found for service '$SERVICE'."
else
  record_check "container_running" "SKIP" "Container status was not checked because Docker daemon access failed."
fi

if [[ "$DOCKER_ACCESS_STATUS" != "PASS" ]]; then
  record_check "docker_auth" "SKIP" "Registry auth probe was skipped because Docker daemon access failed."
elif (( SKIP_DOCKER_AUTH_CHECK )); then
  DOCKER_AUTH_STATUS="SKIP"
  record_check "docker_auth" "SKIP" "Skipped Docker registry auth probe for '$BASE_IMAGE'."
else
  if auth_output="$(timeout 30 docker buildx imagetools inspect "$BASE_IMAGE" 2>&1)"; then
    DOCKER_AUTH_STATUS="PASS"
    record_check "docker_auth" "PASS" "Registry metadata for '$BASE_IMAGE' is readable."
  else
    DOCKER_AUTH_STATUS="WARN"
    auth_output="$(summarize_output "$auth_output")"
    record_check "docker_auth" "WARN" "Registry metadata probe failed: ${auth_output}"
  fi
fi

if [[ "$IMAGE_PRESENT_STATUS" == "PASS" ]]; then
  image_created="$(docker image inspect --format '{{.Created}}' "$IMAGE" 2>/dev/null || true)"
  image_created_epoch="$(date -d "$image_created" +%s 2>/dev/null || true)"
  latest_baseline_input_epoch="$(baseline_input_latest_epoch || true)"
  latest_baseline_input_epoch="${latest_baseline_input_epoch%%.*}"
  if [[ -n "$image_created_epoch" && -n "$latest_baseline_input_epoch" ]] && (( image_created_epoch >= latest_baseline_input_epoch )); then
    IMAGE_FRESHNESS_STATUS="PASS"
    record_check "image_freshness" "PASS" "Image '$IMAGE' is newer than the latest baseline image-owned source change."
  elif [[ -n "$image_created_epoch" && -n "$latest_baseline_input_epoch" ]]; then
    IMAGE_FRESHNESS_STATUS="FAIL"
    record_check "image_freshness" "FAIL" "Image '$IMAGE' is older than the latest baseline image-owned source change. Rebuild the image before running."
  else
    IMAGE_FRESHNESS_STATUS="WARN"
    record_check "image_freshness" "WARN" "Could not determine whether image '$IMAGE' is newer than the latest baseline image-owned source change."
  fi
elif [[ "$DOCKER_ACCESS_STATUS" == "PASS" ]]; then
  record_check "image_freshness" "SKIP" "Image freshness was not checked because '$IMAGE' is missing."
else
  record_check "image_freshness" "SKIP" "Image freshness was not checked because Docker daemon access failed."
fi

if [[ "$IMAGE_PRESENT_STATUS" == "PASS" && "$CONTAINER_RUNNING_STATUS" == "PASS" ]]; then
  local_image_id="$(docker image inspect --format '{{.Id}}' "$IMAGE")"
  container_image_id="$(docker inspect --format '{{.Image}}' "$CONTAINER_REF")"
  if [[ "$local_image_id" == "$container_image_id" ]]; then
    CONTAINER_IMAGE_STATUS="PASS"
    record_check "container_image" "PASS" "Running container matches the current local image '$IMAGE'."
  else
    CONTAINER_IMAGE_STATUS="FAIL"
    record_check "container_image" "FAIL" "Running container does not match the current local image '$IMAGE'. Recreate the container after rebuilding or retagging."
  fi
elif [[ "$DOCKER_ACCESS_STATUS" != "PASS" ]]; then
  record_check "container_image" "SKIP" "Container-image match was not checked because Docker daemon access failed."
else
  record_check "container_image" "SKIP" "Container-image match was not checked because the image or running container is unavailable."
fi

skill_rel_path="$(relative_to_root "$SKILL_PATH")"
skill_checksum="$(dir_tree_hash "$SKILL_PATH")"
skill_commit="$(git -C "$ROOT_DIR" log -1 --format=%H -- "$skill_rel_path" 2>/dev/null || true)"
skill_dirty="$(git -C "$ROOT_DIR" status --porcelain -- "$skill_rel_path" 2>/dev/null || true)"
if [[ -z "$skill_checksum" ]]; then
  SKILL_CHECKSUM_STATUS="FAIL"
  record_check "skill_checksum" "FAIL" "Could not compute a canonical checksum for '$skill_rel_path'."
elif [[ -n "$REQUIRED_SKILL_CHECKSUM" ]] && [[ "$skill_checksum" != "$REQUIRED_SKILL_CHECKSUM" ]]; then
  SKILL_CHECKSUM_STATUS="WARN"
  record_check "skill_checksum" "WARN" "Skill tree checksum is '$skill_checksum', expected '$REQUIRED_SKILL_CHECKSUM'."
else
  SKILL_CHECKSUM_STATUS="PASS"
  record_check "skill_checksum" "PASS" "Skill tree checksum is '$skill_checksum'."
fi

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

if (( ${#RUNTIME_PACKAGES[@]} > 0 )); then
  PACKAGE_SYNC_STATUS="PASS"
  PACKAGE_BUILD_STATUS="PASS"
  for runtime_package in "${RUNTIME_PACKAGES[@]}"; do
    package_sync_check_name="package_sync:${runtime_package}"
    package_build_check_name="package_build_sync:${runtime_package}"
    package_sync_result="SKIP"
    package_build_result="SKIP"
    host_package_dir="${ROOT_DIR}/src/${runtime_package}"

    if [[ ! -d "$host_package_dir" ]]; then
      package_sync_result="FAIL"
      record_check "$package_sync_check_name" "FAIL" "Host package '${runtime_package}' does not exist under src/."
    elif [[ "$DOCKER_ACCESS_STATUS" != "PASS" ]]; then
      record_check "$package_sync_check_name" "SKIP" "Package sync was not checked because Docker daemon access failed."
    elif [[ "$CONTAINER_RUNNING_STATUS" != "PASS" ]]; then
      package_sync_result="FAIL"
      record_check "$package_sync_check_name" "FAIL" "Cannot verify /ws/src sync because the container is not running."
    elif ! docker exec "$CONTAINER_REF" bash -lc "test -d /ws/src/'$runtime_package'" >/dev/null 2>&1; then
      package_sync_result="FAIL"
      record_check "$package_sync_check_name" "FAIL" "Package '${runtime_package}' is not present inside /ws/src."
    else
      host_hash="$(scenario_dir_hash "${ROOT_DIR}/src" "$runtime_package")"
      container_hash="$(scenario_dir_hash_in_container "$CONTAINER_REF" "$runtime_package")"
      if [[ "$host_hash" == "$container_hash" ]]; then
        package_sync_result="PASS"
        record_check "$package_sync_check_name" "PASS" "Host and container copies of '${runtime_package}' match."
      else
        package_sync_result="FAIL"
        record_check "$package_sync_check_name" "FAIL" "Host and container copies of '${runtime_package}' differ. Copy the package into the container again before building or launching."
      fi
    fi

    if [[ "$package_sync_result" != "PASS" ]]; then
      record_check "$package_build_check_name" "SKIP" "Installed-package freshness was not checked because /ws/src package sync did not pass."
    else
      if ! docker exec "$CONTAINER_REF" bash -lc "test -f /ws/install/share/'$runtime_package'/package.dsv -o -f /ws/install/share/'$runtime_package'/package.xml" >/dev/null 2>&1; then
        package_build_result="FAIL"
        record_check "$package_build_check_name" "FAIL" "Installed package markers for '${runtime_package}' are missing under /ws/install/share. Rebuild the package before launching."
      else
        package_source_mtime="$(package_latest_mtime_in_container "$CONTAINER_REF" "$runtime_package")"
        package_install_mtime="$(package_install_marker_mtime_in_container "$CONTAINER_REF" "$runtime_package")"
        if awk -v source_time="$package_source_mtime" -v install_time="$package_install_mtime" 'BEGIN { exit !(install_time + 0 >= source_time + 0) }'; then
          package_build_result="PASS"
          record_check "$package_build_check_name" "PASS" "Installed package markers for '${runtime_package}' are at least as new as the container source tree."
        else
          package_build_result="FAIL"
          record_check "$package_build_check_name" "FAIL" "Container source for '${runtime_package}' is newer than its installed package markers. Rebuild the package after copying it into /ws/src."
        fi
      fi
    fi

    if [[ "$package_sync_result" != "PASS" ]]; then
      PACKAGE_SYNC_STATUS="FAIL"
    fi
    if [[ "$package_build_result" != "PASS" ]]; then
      PACKAGE_BUILD_STATUS="FAIL"
    fi
  done
else
  record_check "package_sync" "WARN" "No runtime package was provided, so /ws/src sync was not checked."
  record_check "package_build_sync" "WARN" "No runtime package was provided, so installed-package freshness was not checked."
fi

main_run_ready=1
main_run_reasons=()

if [[ "$RUN_CLASS" != "full_run" ]]; then
  main_run_ready=0
  main_run_reasons+=("run_class=${RUN_CLASS} is not eligible to serve as the primary full_run result")
fi

if [[ "$DOCKER_ACCESS_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("docker daemon access did not pass")
fi

if [[ "$IMAGE_PRESENT_STATUS" == "FAIL" ]]; then
  main_run_ready=0
  main_run_reasons+=("local image '${IMAGE}' is missing")
fi

if [[ "$IMAGE_FRESHNESS_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("image freshness check did not pass")
fi

if [[ -n "$REQUIRED_SKILL_CHECKSUM" ]] && [[ "$SKILL_CHECKSUM_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("required skill checksum pinning did not pass")
fi

if [[ -n "$REQUIRED_SKILL_COMMIT" ]] && [[ "$SKILL_COMMIT_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("required skill commit pinning did not pass")
fi

if [[ "$CONTAINER_RUNNING_STATUS" == "FAIL" ]]; then
  main_run_ready=0
  main_run_reasons+=("scenario container is not running")
fi

if [[ "$CONTAINER_IMAGE_STATUS" == "FAIL" ]]; then
  main_run_ready=0
  main_run_reasons+=("running container does not match the current local image")
fi

if (( ${#RUNTIME_PACKAGES[@]} == 0 )); then
  main_run_ready=0
  main_run_reasons+=("no runtime package was provided, so sync eligibility could not be checked")
elif [[ "$PACKAGE_SYNC_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("host-vs-container runtime package sync did not pass")
fi

if (( ${#RUNTIME_PACKAGES[@]} > 0 )) && [[ "$PACKAGE_BUILD_STATUS" != "PASS" ]]; then
  main_run_ready=0
  main_run_reasons+=("one or more installed runtime packages are older than the current /ws/src copies")
fi

printf 'Scenario preflight summary\n'
printf '  repo_root: %s\n' "$ROOT_DIR"
printf '  run_class: %s\n' "$RUN_CLASS"
printf '  image: %s\n' "$IMAGE"
printf '  service: %s\n' "$SERVICE"
if [[ -n "$SCENARIO_PACKAGE" ]]; then
  printf '  scenario_package: %s\n' "$SCENARIO_PACKAGE"
fi
if (( ${#RUNTIME_PACKAGES[@]} > 0 )); then
  printf '  runtime_packages: %s\n' "${RUNTIME_PACKAGES[*]}"
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
