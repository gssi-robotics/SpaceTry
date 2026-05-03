#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
COMPOSE_FILE="${ROOT_DIR}/docker/docker-compose.yaml"
SERVICE="spacetry"
LAUNCH_PACKAGE=""
LAUNCH_FILE=""
SCENARIO_PACKAGE=""
RUNTIME_PACKAGES=()
RUN_CLASS="full_run"
RUN_LABEL=""
OUTPUT_ROOT=""
RECORD_ROSBAG="true"
HEADLESS="1"
BATTERY=""
INTERRUPT_AFTER=""
REQUIRED_SKILL_CHECKSUM=""
REQUIRED_SKILL_COMMIT=""
REQUIRE_MAIN_RUN_READY=0
SKIP_PREFLIGHT=0
ALLOWED_FULL_RUN_REASONS=("goal_reached" "timeout")
LAUNCH_ARGS=()

usage() {
  cat <<'EOF'
Usage: skills/spacetry-autonomy-scenario-driver/scripts/run_scenario_full.sh [options]

Launches a scenario through Docker and classifies the run as full_run, smoke,
or tuning. Only full_run executions that finish naturally with an allowed
termination reason may count as a main result.

Options:
  --launch-package <name>             ROS 2 package passed to ros2 launch
  --launch-file <name>                Launch file passed to ros2 launch
  --scenario-package <name>           Primary scenario package used for preflight
                                      sync checks
  --runtime-package <name>            Additional repo-local runtime package to
                                      validate in preflight, repeat as needed
  --run-class <full_run|smoke|tuning> Run classification
  --run-label <label>                 User label to include after the run class prefix
  --output-root <path>                Container-visible output root, default /ws/log/<scenario-or-launch-package>
  --record-rosbag <true|false>        Pass record_rosbag launch arg
  --headless <0|1>                    Pass headless launch arg
  --battery <soc>                     Pass battery launch arg when needed
  --interrupt-after <seconds>         Intentionally interrupt smoke/tuning runs after N seconds
  --launch-arg <name:=value>          Additional launch arg, repeat as needed
  --allowed-full-run-reason <name>    Additional termination reason accepted for full_run
  --required-skill-checksum <sha>     Skill tree checksum required when exact
                                      skill-state pinning matters
  --required-skill-commit <sha>       Commit hash required when main-run pinning matters
  --require-main-run-ready            Require main-run-ready preflight before launch
                                      (already implied for full_run)
  --skip-preflight                    Skip skills/spacetry-autonomy-scenario-driver/scripts/scenario_preflight.sh
  --service <name>                    Docker Compose service name
  --compose-file <path>               Compose file to use
  --help                              Show this help
EOF
}

sanitize_label() {
  printf '%s\n' "$1" | tr -cs 'A-Za-z0-9_.-' '_'
}

container_to_host_output_root() {
  local container_root="$1"
  if [[ "$container_root" == /ws/log* ]]; then
    printf '%s%s\n' "${ROOT_DIR}/log" "${container_root#/ws/log}"
  fi
}

json_top_level_field() {
  local json_file="$1"
  local key="$2"
  python3 - "$json_file" "$key" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    data = json.load(handle)

value = data.get(sys.argv[2], "")
if isinstance(value, (dict, list)):
    print(json.dumps(value, sort_keys=True))
elif value is None:
    print("")
else:
    print(value)
PY
}

for required_cmd in docker python3 date find; do
  if ! command -v "$required_cmd" >/dev/null 2>&1; then
    echo "Missing required command: $required_cmd" >&2
    exit 1
  fi
done

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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --launch-package)
      LAUNCH_PACKAGE="$2"
      shift 2
      ;;
    --launch-file)
      LAUNCH_FILE="$2"
      shift 2
      ;;
    --scenario-package)
      SCENARIO_PACKAGE="$2"
      shift 2
      ;;
    --runtime-package)
      append_unique_runtime_package "$2"
      shift 2
      ;;
    --run-class)
      RUN_CLASS="$2"
      shift 2
      ;;
    --run-label)
      RUN_LABEL="$2"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --record-rosbag)
      RECORD_ROSBAG="$2"
      shift 2
      ;;
    --headless)
      HEADLESS="$2"
      shift 2
      ;;
    --battery)
      BATTERY="$2"
      shift 2
      ;;
    --interrupt-after)
      INTERRUPT_AFTER="$2"
      shift 2
      ;;
    --launch-arg)
      LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --allowed-full-run-reason)
      ALLOWED_FULL_RUN_REASONS+=("$2")
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
    --require-main-run-ready)
      REQUIRE_MAIN_RUN_READY=1
      shift
      ;;
    --skip-preflight)
      SKIP_PREFLIGHT=1
      shift
      ;;
    --service)
      SERVICE="$2"
      shift 2
      ;;
    --compose-file)
      COMPOSE_FILE="$2"
      shift 2
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

if [[ -z "$LAUNCH_PACKAGE" || -z "$LAUNCH_FILE" ]]; then
  echo "--launch-package and --launch-file are required." >&2
  exit 2
fi

if [[ -n "$SCENARIO_PACKAGE" ]]; then
  append_unique_runtime_package "$SCENARIO_PACKAGE"
fi

if [[ -n "$INTERRUPT_AFTER" && "$RUN_CLASS" == "full_run" ]]; then
  echo "--interrupt-after is only allowed for smoke or tuning runs." >&2
  exit 2
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
label_suffix="$(sanitize_label "${RUN_LABEL:-${LAUNCH_PACKAGE}}")"
label_suffix="${label_suffix#_}"
if [[ -n "$label_suffix" ]]; then
  EFFECTIVE_RUN_LABEL="${RUN_CLASS}_${label_suffix}_${timestamp}"
else
  EFFECTIVE_RUN_LABEL="${RUN_CLASS}_${timestamp}"
fi

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_BASE="${SCENARIO_PACKAGE:-$LAUNCH_PACKAGE}"
  OUTPUT_ROOT="/ws/log/${OUTPUT_BASE}"
fi

HOST_OUTPUT_ROOT="$(container_to_host_output_root "$OUTPUT_ROOT" || true)"
HOST_RUN_DIR=""
SUMMARY_PATH=""
if [[ -n "$HOST_OUTPUT_ROOT" ]]; then
  HOST_RUN_DIR="${HOST_OUTPUT_ROOT}/${EFFECTIVE_RUN_LABEL}"
  SUMMARY_PATH="${HOST_RUN_DIR}/runtime/wrapper_run_summary.txt"
  mkdir -p "${HOST_RUN_DIR}/runtime"
fi

if (( ! SKIP_PREFLIGHT )); then
  preflight_cmd=(
    "${SCRIPT_DIR}/scenario_preflight.sh"
    --compose-file "$COMPOSE_FILE"
    --service "$SERVICE"
    --run-class "$RUN_CLASS"
  )
  if [[ -n "$SCENARIO_PACKAGE" ]]; then
    preflight_cmd+=(--scenario-package "$SCENARIO_PACKAGE")
  fi
  for runtime_package in "${RUNTIME_PACKAGES[@]}"; do
    preflight_cmd+=(--runtime-package "$runtime_package")
  done
  if [[ -n "$REQUIRED_SKILL_CHECKSUM" ]]; then
    preflight_cmd+=(--required-skill-checksum "$REQUIRED_SKILL_CHECKSUM")
  fi
  if [[ -n "$REQUIRED_SKILL_COMMIT" ]]; then
    preflight_cmd+=(--required-skill-commit "$REQUIRED_SKILL_COMMIT")
  fi
  if [[ "$RUN_CLASS" == "full_run" ]] || (( REQUIRE_MAIN_RUN_READY )); then
    preflight_cmd+=(--require-main-run-ready)
  fi
  "${preflight_cmd[@]}"
fi

launch_parts=(ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE")
launch_parts+=("headless:=${HEADLESS}")
launch_parts+=("output_root:=${OUTPUT_ROOT}")
launch_parts+=("run_label:=${EFFECTIVE_RUN_LABEL}")
launch_parts+=("record_rosbag:=${RECORD_ROSBAG}")
if [[ -n "$BATTERY" ]]; then
  launch_parts+=("battery:=${BATTERY}")
fi
for extra_arg in "${LAUNCH_ARGS[@]}"; do
  launch_parts+=("$extra_arg")
done

launch_ros_cmd="$(printf '%q ' "${launch_parts[@]}")"
container_cmd="source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && ${launch_ros_cmd}"

wrapper_signal=""
launch_status=0
termination_reason=""
metrics_file=""
main_result_eligible="no"

forward_interrupt() {
  wrapper_signal="$1"
  echo "Forwarding ${wrapper_signal} to the active scenario launch..."
  if [[ -n "${launch_pid:-}" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
  fi
}

trap 'forward_interrupt INT' INT
trap 'forward_interrupt TERM' TERM

echo "Launching ${LAUNCH_PACKAGE}/${LAUNCH_FILE}"
echo "  run_class: ${RUN_CLASS}"
echo "  run_label: ${EFFECTIVE_RUN_LABEL}"
echo "  output_root: ${OUTPUT_ROOT}"

docker compose -f "$COMPOSE_FILE" exec -T "$SERVICE" bash -lc "$container_cmd" &
launch_pid=$!

if [[ -n "$INTERRUPT_AFTER" ]]; then
  (
    sleep "$INTERRUPT_AFTER"
    echo "Interrupting ${RUN_CLASS} run after ${INTERRUPT_AFTER}s"
    kill -INT "$launch_pid" 2>/dev/null || true
  ) &
  interrupter_pid=$!
else
  interrupter_pid=""
fi

set +e
wait "$launch_pid"
launch_status=$?
set -e

if [[ -n "$interrupter_pid" ]]; then
  kill "$interrupter_pid" 2>/dev/null || true
fi

if [[ -n "$HOST_RUN_DIR" && -d "$HOST_RUN_DIR" ]]; then
  metrics_file="$(find "${HOST_RUN_DIR}" -maxdepth 4 -type f -name '*_metrics.json' | sort | head -n 1)"
fi

if [[ -n "$metrics_file" ]]; then
  termination_reason="$(json_top_level_field "$metrics_file" termination_reason)"
fi

if [[ "$RUN_CLASS" == "full_run" && -z "$wrapper_signal" && "$launch_status" -eq 0 ]]; then
  for allowed_reason in "${ALLOWED_FULL_RUN_REASONS[@]}"; do
    if [[ "$termination_reason" == "$allowed_reason" ]]; then
      main_result_eligible="yes"
      break
    fi
  done
fi

echo
echo "Run summary"
echo "  launch_status: ${launch_status}"
if [[ -n "$termination_reason" ]]; then
  echo "  termination_reason: ${termination_reason}"
fi
echo "  main_result_eligible: ${main_result_eligible}"
if [[ "$RUN_CLASS" != "full_run" ]]; then
  echo "  note: ${RUN_CLASS} runs are labeled as non-main results by design."
elif [[ "$main_result_eligible" != "yes" ]]; then
  echo "  note: this full_run did not finish with an allowed natural termination reason."
fi

if [[ -n "$SUMMARY_PATH" ]]; then
  {
    printf 'run_class=%s\n' "$RUN_CLASS"
    printf 'run_label=%s\n' "$EFFECTIVE_RUN_LABEL"
    printf 'launch_package=%s\n' "$LAUNCH_PACKAGE"
    printf 'launch_file=%s\n' "$LAUNCH_FILE"
    printf 'launch_status=%s\n' "$launch_status"
    printf 'wrapper_signal=%s\n' "${wrapper_signal:-}"
    printf 'termination_reason=%s\n' "$termination_reason"
    printf 'main_result_eligible=%s\n' "$main_result_eligible"
    printf 'require_main_run_ready=%s\n' "$REQUIRE_MAIN_RUN_READY"
    printf 'metrics_file=%s\n' "$metrics_file"
  } >"$SUMMARY_PATH"
  echo "  wrapper_summary: ${SUMMARY_PATH}"
fi

if [[ "$RUN_CLASS" == "full_run" && "$main_result_eligible" != "yes" ]]; then
  exit 1
fi

if [[ "$RUN_CLASS" != "full_run" && ( -n "$INTERRUPT_AFTER" || -n "$wrapper_signal" ) ]]; then
  exit 0
fi

exit "$launch_status"
