#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/env.sh"

MAIN_GENERATOR="${SHAPE_ASSEMBLY_MAIN_GENERATOR:-${SCRIPT_DIR}/../src/plugins/dynamic_xml_config/main_generate_simtest.py}"
USER_CONFIG_BASENAME="${SHAPE_ASSEMBLY_USER_CONFIG_BASENAME:-user_config_5x10_d2_simtest.yaml}"
TARGET_LAUNCH="${SHAPE_ASSEMBLY_TARGET_LAUNCH:-shape_assembly_with_main_simtest.launch}"
USER_CONFIG_PATH="${SCRIPT_DIR}/../src/user_config/${USER_CONFIG_BASENAME}"
SHAPE_CONFIG_PATH="${SHAPE_ASSEMBLY_SHAPE_CONFIG_PATH:-${SCRIPT_DIR}/../src/sim_env/config/shape_assembly_simtest.yaml}"

LOCK_FILE="/tmp/shape_assembly_5x10_d2.lock"

_acquire_lock() {
  if [[ -f "${LOCK_FILE}" ]]; then
    old_pid="$(cat "${LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
      echo "shape_assembly_5x10_d2.sh is already running (pid=${old_pid})."
      echo "Stop the previous run first (Ctrl+C), then retry."
      exit 1
    fi
    rm -f "${LOCK_FILE}" || true
  fi
  echo "$$" > "${LOCK_FILE}"
}

_release_lock() {
  rm -f "${LOCK_FILE}" || true
}

_kill_pattern() {
  local pattern="$1"
  pkill -f "${pattern}" >/dev/null 2>&1 || true
  sleep 0.3
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
  fi
}

_acquire_lock
trap _release_lock EXIT INT TERM

if [[ "${SKIP_5X10_D2_CLEANUP:-0}" != "1" ]]; then
  if command -v rosnode >/dev/null 2>&1; then
    rosnode kill /gazebo /gazebo_gui >/dev/null 2>&1 || true

    if rosnode list >/dev/null 2>&1; then
      if command -v rg >/dev/null 2>&1; then
        old_nodes=$(rosnode list | rg '^/robot[0-9]+/' || true)
      else
        old_nodes=$(rosnode list | grep -E '^/robot[0-9]+/' || true)
      fi
      if [[ -n "${old_nodes}" ]]; then
        while IFS= read -r n; do
          [[ -z "${n}" ]] && continue
          rosnode kill "${n}" >/dev/null 2>&1 || true
        done <<< "${old_nodes}"
      fi
    fi

    yes | rosnode cleanup >/dev/null 2>&1 || true
  fi

  _kill_pattern 'shape_assembly_with_main(_simtest)?\.launch'
  _kill_pattern '/controller_manager/spawner'
  _kill_pattern '/robot_state_publisher/robot_state_publisher'
  _kill_pattern '/opt/ros/.*/bin/roslaunch sim_env shape_assembly_with_main(_simtest)?\.launch'
  _kill_pattern 'gzserver.*sim_env/worlds/'
  _kill_pattern '/opt/ros/.*/lib/gazebo_ros/gzserver'
  _kill_pattern '/opt/ros/.*/lib/gazebo_ros/gzclient'
  _kill_pattern 'gzclient -g '

  sleep 2
fi

# Generate the simtest environment launch from user config (world/map/robots).
python "${MAIN_GENERATOR}" "${USER_CONFIG_BASENAME}"

DEFAULT_AGENT_NUMBER="$(rg -c 'robot[0-9]+_type' "${USER_CONFIG_PATH}" 2>/dev/null || true)"
if ! [[ "${DEFAULT_AGENT_NUMBER}" =~ ^[0-9]+$ ]] || [[ "${DEFAULT_AGENT_NUMBER}" -le 0 ]]; then
  DEFAULT_AGENT_NUMBER=4
fi

_yaml_get() {
  local key="$1"
  python - "${SHAPE_CONFIG_PATH}" "${key}" <<'PY'
import json
import sys
try:
    import yaml
except Exception:
    sys.exit(1)

path = sys.argv[1]
key = sys.argv[2]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
except Exception:
    sys.exit(1)

if key not in data:
    sys.exit(1)

value = data[key]
if isinstance(value, bool):
    print("true" if value else "false")
elif isinstance(value, (int, float)):
    print(value)
elif isinstance(value, str):
    print(value)
else:
    print(json.dumps(value, ensure_ascii=False, separators=(",", ":")))
PY
}

_yaml_get_first() {
  local value=""
  local key=""
  for key in "$@"; do
    if value="$(_yaml_get "${key}" 2>/dev/null)"; then
      printf '%s\n' "${value}"
      return 0
    fi
  done
  return 1
}

HAS_AGENT_NUMBER=0
HAS_NAMESPACE_PREFIX=0
HAS_USE_FM2=0
HAS_ENABLE_MAP_COMBINE=0
HAS_CENTER_ONLY=0
HAS_CENTER_TOPIC=0
HAS_FM2_AUTO_DETECT=0
HAS_FM2_TOPIC_SUFFIX=0
HAS_FM2_TIMEOUT=0
HAS_REPLAN_MODE=0
HAS_PERIODIC_INTERVAL=0
HAS_COOLDOWN=0
HAS_PER_ROBOT_COOLDOWN=0
HAS_MAX_LEN_GROWTH_RATIO=0
HAS_LONG_PATH_REPLAN_MIN_LENGTH=0
HAS_REPLAN_GROWTH_METRIC=0
HAS_GATHER_COST_SOURCE=0
HAS_ARRIVAL_TIME_TOPIC_SUFFIX=0
HAS_GATHER_COST_GROWTH_RATIO=0
HAS_MIN_GATHER_COST_GROWTH_TRIGGER=0
HAS_ENABLE_EMPTY_PLAN_REPLAN=0
HAS_EMPTY_PLAN_REPLAN_COUNT=0
HAS_FORMAT_RADIUS=0
HAS_ENABLE_STABLE_LEN_REPLAN=0
HAS_GOAL_OCCUPIED_PEER_FALLBACK=0
HAS_GOAL_OCCUPIED_RADIUS=0
HAS_DISTRIBUTED_MODE=0

for arg in "$@"; do
  if [[ "${arg}" == agent_number:=* ]]; then
    HAS_AGENT_NUMBER=1
  fi
  if [[ "${arg}" == namespace_prefix:=* ]]; then
    HAS_NAMESPACE_PREFIX=1
  fi
  if [[ "${arg}" == use_fm2_gather:=* ]]; then
    HAS_USE_FM2=1
  fi
  if [[ "${arg}" == enable_map_combine:=* ]]; then
    HAS_ENABLE_MAP_COMBINE=1
  fi
  if [[ "${arg}" == publish_center_goal_only:=* ]]; then
    HAS_CENTER_ONLY=1
  fi
  if [[ "${arg}" == gather_center_topic:=* ]]; then
    HAS_CENTER_TOPIC=1
  fi
  if [[ "${arg}" == fm2_auto_detect_robots:=* ]]; then
    HAS_FM2_AUTO_DETECT=1
  fi
  if [[ "${arg}" == fm2_robot_detect_topic_suffix:=* ]]; then
    HAS_FM2_TOPIC_SUFFIX=1
  fi
  if [[ "${arg}" == fm2_robot_detect_timeout:=* ]]; then
    HAS_FM2_TIMEOUT=1
  fi
  if [[ "${arg}" == replan_mode:=* ]]; then
    HAS_REPLAN_MODE=1
  fi
  if [[ "${arg}" == periodic_interval:=* ]]; then
    HAS_PERIODIC_INTERVAL=1
  fi
  if [[ "${arg}" == cooldown:=* ]]; then
    HAS_COOLDOWN=1
  fi
  if [[ "${arg}" == per_robot_cooldown:=* ]]; then
    HAS_PER_ROBOT_COOLDOWN=1
  fi
  if [[ "${arg}" == max_len_growth_ratio:=* ]]; then
    HAS_MAX_LEN_GROWTH_RATIO=1
  fi
  if [[ "${arg}" == long_path_replan_min_length:=* ]]; then
    HAS_LONG_PATH_REPLAN_MIN_LENGTH=1
  fi
  if [[ "${arg}" == replan_growth_metric:=* ]]; then
    HAS_REPLAN_GROWTH_METRIC=1
  fi
  if [[ "${arg}" == gather_cost_source:=* ]]; then
    HAS_GATHER_COST_SOURCE=1
  fi
  if [[ "${arg}" == arrival_time_topic_suffix:=* ]]; then
    HAS_ARRIVAL_TIME_TOPIC_SUFFIX=1
  fi
  if [[ "${arg}" == gather_cost_growth_ratio:=* ]]; then
    HAS_GATHER_COST_GROWTH_RATIO=1
  fi
  if [[ "${arg}" == min_gather_cost_growth_trigger:=* ]]; then
    HAS_MIN_GATHER_COST_GROWTH_TRIGGER=1
  fi
  if [[ "${arg}" == enable_empty_plan_replan:=* ]]; then
    HAS_ENABLE_EMPTY_PLAN_REPLAN=1
  fi
  if [[ "${arg}" == empty_plan_replan_count:=* ]]; then
    HAS_EMPTY_PLAN_REPLAN_COUNT=1
  fi
  if [[ "${arg}" == format_radius:=* ]]; then
    HAS_FORMAT_RADIUS=1
  fi
  if [[ "${arg}" == enable_stable_len_replan:=* ]]; then
    HAS_ENABLE_STABLE_LEN_REPLAN=1
  fi
  if [[ "${arg}" == use_goal_occupied_peer_fallback:=* ]]; then
    HAS_GOAL_OCCUPIED_PEER_FALLBACK=1
  fi
  if [[ "${arg}" == goal_occupied_radius:=* ]]; then
    HAS_GOAL_OCCUPIED_RADIUS=1
  fi
  if [[ "${arg}" == distributed_mode:=* ]]; then
    HAS_DISTRIBUTED_MODE=1
  fi
done

YAML_AGENT_NUMBER="$(_yaml_get_first agent_number num_robots 2>/dev/null || true)"
YAML_NAMESPACE_PREFIX="$(_yaml_get_first namespace_prefix robot_namespace_prefix 2>/dev/null || true)"
YAML_USE_FM2="$(_yaml_get use_fm2_gather 2>/dev/null || true)"
YAML_ENABLE_MAP_COMBINE="$(_yaml_get enable_map_combine 2>/dev/null || true)"
YAML_CENTER_ONLY="$(_yaml_get publish_center_goal_only 2>/dev/null || true)"
YAML_CENTER_TOPIC="$(_yaml_get gather_center_topic 2>/dev/null || true)"
YAML_FM2_AUTO_DETECT="$(_yaml_get_first fm2_auto_detect_robots auto_detect_robots 2>/dev/null || true)"
YAML_FM2_TOPIC_SUFFIX="$(_yaml_get_first fm2_robot_detect_topic_suffix robot_detect_topic_suffix 2>/dev/null || true)"
YAML_FM2_TIMEOUT="$(_yaml_get_first fm2_robot_detect_timeout robot_detect_timeout 2>/dev/null || true)"
YAML_REPLAN_MODE="$(_yaml_get replan_mode 2>/dev/null || true)"
YAML_PERIODIC_INTERVAL="$(_yaml_get periodic_interval 2>/dev/null || true)"
YAML_COOLDOWN="$(_yaml_get cooldown 2>/dev/null || true)"
YAML_PER_ROBOT_COOLDOWN="$(_yaml_get per_robot_cooldown 2>/dev/null || true)"
YAML_MAX_LEN_GROWTH_RATIO="$(_yaml_get max_len_growth_ratio 2>/dev/null || true)"
YAML_LONG_PATH_REPLAN_MIN_LENGTH="$(_yaml_get long_path_replan_min_length 2>/dev/null || true)"
YAML_REPLAN_GROWTH_METRIC="$(_yaml_get replan_growth_metric 2>/dev/null || true)"
YAML_GATHER_COST_SOURCE="$(_yaml_get gather_cost_source 2>/dev/null || true)"
YAML_ARRIVAL_TIME_TOPIC_SUFFIX="$(_yaml_get arrival_time_topic_suffix 2>/dev/null || true)"
YAML_GATHER_COST_GROWTH_RATIO="$(_yaml_get gather_cost_growth_ratio 2>/dev/null || true)"
YAML_MIN_GATHER_COST_GROWTH_TRIGGER="$(_yaml_get min_gather_cost_growth_trigger 2>/dev/null || true)"
YAML_ENABLE_EMPTY_PLAN_REPLAN="$(_yaml_get enable_empty_plan_replan 2>/dev/null || true)"
YAML_EMPTY_PLAN_REPLAN_COUNT="$(_yaml_get empty_plan_replan_count 2>/dev/null || true)"
YAML_FORMAT_RADIUS="$(_yaml_get format_radius 2>/dev/null || true)"
YAML_ENABLE_STABLE_LEN_REPLAN="$(_yaml_get enable_stable_len_replan 2>/dev/null || true)"
YAML_GOAL_OCCUPIED_PEER_FALLBACK="$(_yaml_get use_goal_occupied_peer_fallback 2>/dev/null || true)"
YAML_GOAL_OCCUPIED_RADIUS="$(_yaml_get goal_occupied_radius 2>/dev/null || true)"
YAML_DISTRIBUTED_MODE="$(_yaml_get distributed_mode 2>/dev/null || true)"

launch_args=("$@")
# Shape-assembly controller parameters are sourced from sim_env/config/shape_assembly.yaml.
# Keep script-level launch args focused on environment/fm2 orchestration.
if [[ "${HAS_AGENT_NUMBER}" -eq 0 ]]; then
  if [[ "${YAML_AGENT_NUMBER}" =~ ^[0-9]+$ ]] && [[ "${YAML_AGENT_NUMBER}" -gt 0 ]]; then
    launch_args=("agent_number:=${YAML_AGENT_NUMBER}" "${launch_args[@]}")
  else
    launch_args=("agent_number:=${DEFAULT_AGENT_NUMBER}" "${launch_args[@]}")
  fi
fi
if [[ "${HAS_NAMESPACE_PREFIX}" -eq 0 ]] && [[ -n "${YAML_NAMESPACE_PREFIX}" ]]; then
  launch_args=("namespace_prefix:=${YAML_NAMESPACE_PREFIX}" "${launch_args[@]}")
fi
if [[ "${HAS_USE_FM2}" -eq 0 ]] && [[ -n "${YAML_USE_FM2}" ]]; then
  launch_args=("use_fm2_gather:=${YAML_USE_FM2}" "${launch_args[@]}")
fi
if [[ "${HAS_ENABLE_MAP_COMBINE}" -eq 0 ]] && [[ -n "${YAML_ENABLE_MAP_COMBINE}" ]]; then
  launch_args=("enable_map_combine:=${YAML_ENABLE_MAP_COMBINE}" "${launch_args[@]}")
fi
if [[ "${HAS_CENTER_ONLY}" -eq 0 ]] && [[ -n "${YAML_CENTER_ONLY}" ]]; then
  launch_args=("publish_center_goal_only:=${YAML_CENTER_ONLY}" "${launch_args[@]}")
fi
if [[ "${HAS_CENTER_TOPIC}" -eq 0 ]] && [[ -n "${YAML_CENTER_TOPIC}" ]]; then
  launch_args=("gather_center_topic:=${YAML_CENTER_TOPIC}" "${launch_args[@]}")
fi
if [[ "${HAS_FM2_AUTO_DETECT}" -eq 0 ]] && [[ -n "${YAML_FM2_AUTO_DETECT}" ]]; then
  launch_args=("fm2_auto_detect_robots:=${YAML_FM2_AUTO_DETECT}" "${launch_args[@]}")
fi
if [[ "${HAS_FM2_TOPIC_SUFFIX}" -eq 0 ]] && [[ -n "${YAML_FM2_TOPIC_SUFFIX}" ]]; then
  launch_args=("fm2_robot_detect_topic_suffix:=${YAML_FM2_TOPIC_SUFFIX}" "${launch_args[@]}")
fi
if [[ "${HAS_FM2_TIMEOUT}" -eq 0 ]] && [[ -n "${YAML_FM2_TIMEOUT}" ]]; then
  launch_args=("fm2_robot_detect_timeout:=${YAML_FM2_TIMEOUT}" "${launch_args[@]}")
fi
if [[ "${HAS_REPLAN_MODE}" -eq 0 ]] && [[ -n "${YAML_REPLAN_MODE}" ]]; then
  launch_args=("replan_mode:=${YAML_REPLAN_MODE}" "${launch_args[@]}")
fi
if [[ "${HAS_PERIODIC_INTERVAL}" -eq 0 ]] && [[ -n "${YAML_PERIODIC_INTERVAL}" ]]; then
  launch_args=("periodic_interval:=${YAML_PERIODIC_INTERVAL}" "${launch_args[@]}")
fi
if [[ "${HAS_COOLDOWN}" -eq 0 ]] && [[ -n "${YAML_COOLDOWN}" ]]; then
  launch_args=("cooldown:=${YAML_COOLDOWN}" "${launch_args[@]}")
fi
if [[ "${HAS_PER_ROBOT_COOLDOWN}" -eq 0 ]] && [[ -n "${YAML_PER_ROBOT_COOLDOWN}" ]]; then
  launch_args=("per_robot_cooldown:=${YAML_PER_ROBOT_COOLDOWN}" "${launch_args[@]}")
fi
if [[ "${HAS_MAX_LEN_GROWTH_RATIO}" -eq 0 ]] && [[ -n "${YAML_MAX_LEN_GROWTH_RATIO}" ]]; then
  launch_args=("max_len_growth_ratio:=${YAML_MAX_LEN_GROWTH_RATIO}" "${launch_args[@]}")
fi
if [[ "${HAS_LONG_PATH_REPLAN_MIN_LENGTH}" -eq 0 ]] && [[ -n "${YAML_LONG_PATH_REPLAN_MIN_LENGTH}" ]]; then
  launch_args=("long_path_replan_min_length:=${YAML_LONG_PATH_REPLAN_MIN_LENGTH}" "${launch_args[@]}")
fi
if [[ "${HAS_REPLAN_GROWTH_METRIC}" -eq 0 ]] && [[ -n "${YAML_REPLAN_GROWTH_METRIC}" ]]; then
  launch_args=("replan_growth_metric:=${YAML_REPLAN_GROWTH_METRIC}" "${launch_args[@]}")
fi
if [[ "${HAS_GATHER_COST_SOURCE}" -eq 0 ]] && [[ -n "${YAML_GATHER_COST_SOURCE}" ]]; then
  launch_args=("gather_cost_source:=${YAML_GATHER_COST_SOURCE}" "${launch_args[@]}")
fi
if [[ "${HAS_ARRIVAL_TIME_TOPIC_SUFFIX}" -eq 0 ]] && [[ -n "${YAML_ARRIVAL_TIME_TOPIC_SUFFIX}" ]]; then
  launch_args=("arrival_time_topic_suffix:=${YAML_ARRIVAL_TIME_TOPIC_SUFFIX}" "${launch_args[@]}")
fi
if [[ "${HAS_GATHER_COST_GROWTH_RATIO}" -eq 0 ]] && [[ -n "${YAML_GATHER_COST_GROWTH_RATIO}" ]]; then
  launch_args=("gather_cost_growth_ratio:=${YAML_GATHER_COST_GROWTH_RATIO}" "${launch_args[@]}")
fi
if [[ "${HAS_MIN_GATHER_COST_GROWTH_TRIGGER}" -eq 0 ]] && [[ -n "${YAML_MIN_GATHER_COST_GROWTH_TRIGGER}" ]]; then
  launch_args=("min_gather_cost_growth_trigger:=${YAML_MIN_GATHER_COST_GROWTH_TRIGGER}" "${launch_args[@]}")
fi
if [[ "${HAS_ENABLE_EMPTY_PLAN_REPLAN}" -eq 0 ]] && [[ -n "${YAML_ENABLE_EMPTY_PLAN_REPLAN}" ]]; then
  launch_args=("enable_empty_plan_replan:=${YAML_ENABLE_EMPTY_PLAN_REPLAN}" "${launch_args[@]}")
fi
if [[ "${HAS_EMPTY_PLAN_REPLAN_COUNT}" -eq 0 ]] && [[ -n "${YAML_EMPTY_PLAN_REPLAN_COUNT}" ]]; then
  launch_args=("empty_plan_replan_count:=${YAML_EMPTY_PLAN_REPLAN_COUNT}" "${launch_args[@]}")
fi
if [[ "${HAS_FORMAT_RADIUS}" -eq 0 ]] && [[ -n "${YAML_FORMAT_RADIUS}" ]]; then
  launch_args=("format_radius:=${YAML_FORMAT_RADIUS}" "${launch_args[@]}")
fi
if [[ "${HAS_ENABLE_STABLE_LEN_REPLAN}" -eq 0 ]] && [[ -n "${YAML_ENABLE_STABLE_LEN_REPLAN}" ]]; then
  launch_args=("enable_stable_len_replan:=${YAML_ENABLE_STABLE_LEN_REPLAN}" "${launch_args[@]}")
fi
if [[ "${HAS_GOAL_OCCUPIED_PEER_FALLBACK}" -eq 0 ]] && [[ -n "${YAML_GOAL_OCCUPIED_PEER_FALLBACK}" ]]; then
  launch_args=("use_goal_occupied_peer_fallback:=${YAML_GOAL_OCCUPIED_PEER_FALLBACK}" "${launch_args[@]}")
fi
if [[ "${HAS_GOAL_OCCUPIED_RADIUS}" -eq 0 ]] && [[ -n "${YAML_GOAL_OCCUPIED_RADIUS}" ]]; then
  launch_args=("goal_occupied_radius:=${YAML_GOAL_OCCUPIED_RADIUS}" "${launch_args[@]}")
fi
if [[ "${HAS_DISTRIBUTED_MODE}" -eq 0 ]] && [[ -n "${YAML_DISTRIBUTED_MODE}" ]]; then
  launch_args=("distributed_mode:=${YAML_DISTRIBUTED_MODE}" "${launch_args[@]}")
fi

roslaunch sim_env "${TARGET_LAUNCH}" "${launch_args[@]}"
