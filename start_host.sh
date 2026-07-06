#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"
source "${ROOT_DIR}/src/ros_motion_planning/scripts/env.sh"

ROSCORE_PID=""
HOST_STACK_PID=""

_config_file="${ROOT_DIR}/config/host_start.conf"
_start_roscore=true
_launch_map_server=true
_auto_start_gather=false
_start_gather_delay=2.0
_start_gather_wait_started=5.0
_start_gather_wait_connections=2.0
_start_gather_repeat=3
_start_gather_rate=5.0
_roscore_wait=8.0
_ros_master_uri="${ROS_MASTER_URI:-http://192.168.1.104:11311}"
_ros_ip="${ROS_IP:-}"
_use_sim_time="${USE_SIM_TIME:-false}"
_map_file=""
host_args=(
  "agent_number:=2"
  "shape_type:=rectangle"
  "shape_source:=mat"
  "shape_scale:=1.0"
  "auto_shape_heading:=true"
)

usage() {
  cat <<EOF
Usage: ./start_host.sh [script args] [shape_assembly_host args]

Default config file:
  ${ROOT_DIR}/config/host_start.conf

Script args:
  config:=/abs/or/relative/path.conf
  start_roscore:=true|false
  launch_map_server:=true|false
  map_file:=/abs/or/relative/path.yaml
  auto_start_gather:=true|false
  start_gather_delay:=2.0
  start_gather_wait_started:=5.0
  start_gather_wait_connections:=2.0
  start_gather_repeat:=3
  start_gather_rate:=5.0
  roscore_wait:=8.0
  ros_master_uri:=http://<HOST_IP>:11311
  ros_ip:=<HOST_IP>
  use_sim_time:=true|false

All other args are forwarded to shape_assembly_real_robot.sh.
Default gather behavior is manual: keep auto_start_gather:=false and publish /gather_signal yourself.
Command-line args override config file values.
EOF
}

merge_arg() {
  local candidate="$1"
  if [[ "$candidate" != *":="* ]]; then
    host_args+=("$candidate")
    return
  fi
  local key="${candidate%%:=*}"
  local i
  for i in "${!host_args[@]}"; do
    if [[ "${host_args[$i]}" == "${key}:="* ]]; then
      host_args[$i]="$candidate"
      return
    fi
  done
  host_args+=("$candidate")
}

count_robot_ids_from_arg() {
  local robot_ids_value="$1"
  robot_ids_value="${robot_ids_value#[}"
  robot_ids_value="${robot_ids_value%]}"
  robot_ids_value="${robot_ids_value//[[:space:]]/}"
  if [[ -z "${robot_ids_value}" ]]; then
    echo 0
    return
  fi

  local count=0
  local item
  IFS=',' read -r -a _robot_ids <<< "${robot_ids_value}"
  for item in "${_robot_ids[@]}"; do
    if [[ -n "${item}" ]]; then
      ((count+=1))
    fi
  done
  echo "${count}"
}

synchronize_agent_number_with_robot_ids() {
  local arg
  local robot_ids_value=""
  for arg in "${host_args[@]}"; do
    if [[ "${arg}" == robot_ids:=* ]]; then
      robot_ids_value="${arg#*=}"
    fi
  done

  if [[ -z "${robot_ids_value}" || "${robot_ids_value}" == "USE_YAML_SENTINEL" ]]; then
    return
  fi

  local robot_count
  robot_count="$(count_robot_ids_from_arg "${robot_ids_value}")"
  if [[ "${robot_count}" =~ ^[0-9]+$ ]] && (( robot_count > 0 )); then
    merge_arg "agent_number:=${robot_count}"
  fi
}

for arg in "$@"; do
  case "$arg" in
    --help|-h)
      usage
      exit 0
      ;;
    config:=*)
      _config_file="${arg#*=}"
      ;;
    --config=*)
      _config_file="${arg#*=}"
      ;;
  esac
done

if [[ -f "$_config_file" ]]; then
  # shellcheck disable=SC1090
  source "$_config_file"
else
  echo "[start_host] Warning: config file not found: $_config_file" >&2
fi

if [[ ${START_ROSCORE+x} ]]; then
  _start_roscore="$START_ROSCORE"
fi
if [[ ${LAUNCH_MAP_SERVER+x} ]]; then
  _launch_map_server="$LAUNCH_MAP_SERVER"
fi
if [[ ${AUTO_START_GATHER+x} ]]; then
  _auto_start_gather="$AUTO_START_GATHER"
fi
if [[ ${ROSCORE_WAIT+x} ]]; then
  _roscore_wait="$ROSCORE_WAIT"
fi
if [[ ${ROS_MASTER_URI_VALUE+x} ]]; then
  _ros_master_uri="$ROS_MASTER_URI_VALUE"
fi
if [[ ${ROS_IP_VALUE+x} ]]; then
  _ros_ip="$ROS_IP_VALUE"
fi
if [[ ${USE_SIM_TIME_VALUE+x} ]]; then
  _use_sim_time="$USE_SIM_TIME_VALUE"
fi
if [[ ${MAP_FILE_VALUE+x} ]]; then
  _map_file="$MAP_FILE_VALUE"
fi
if [[ ${START_GATHER_DELAY+x} ]]; then
  _start_gather_delay="$START_GATHER_DELAY"
fi
if [[ ${START_GATHER_WAIT_STARTED+x} ]]; then
  _start_gather_wait_started="$START_GATHER_WAIT_STARTED"
fi
if [[ ${START_GATHER_WAIT_CONNECTIONS+x} ]]; then
  _start_gather_wait_connections="$START_GATHER_WAIT_CONNECTIONS"
fi
if [[ ${START_GATHER_REPEAT+x} ]]; then
  _start_gather_repeat="$START_GATHER_REPEAT"
fi
if [[ ${START_GATHER_RATE+x} ]]; then
  _start_gather_rate="$START_GATHER_RATE"
fi
if declare -p HOST_LAUNCH_ARGS >/dev/null 2>&1; then
  host_args=("${HOST_LAUNCH_ARGS[@]}")
fi

for arg in "$@"; do
  case "$arg" in
    --help|-h|config:=*|--config=*)
      ;;
    start_roscore:=*)
      _start_roscore="${arg#*=}"
      ;;
    launch_map_server:=*)
      _launch_map_server="${arg#*=}"
      ;;
    map_file:=*)
      _map_file="${arg#*=}"
      ;;
    auto_start_gather:=*)
      _auto_start_gather="${arg#*=}"
      ;;
    start_gather_delay:=*)
      _start_gather_delay="${arg#*=}"
      ;;
    start_gather_wait_started:=*)
      _start_gather_wait_started="${arg#*=}"
      ;;
    start_gather_wait_connections:=*)
      _start_gather_wait_connections="${arg#*=}"
      ;;
    start_gather_repeat:=*)
      _start_gather_repeat="${arg#*=}"
      ;;
    start_gather_rate:=*)
      _start_gather_rate="${arg#*=}"
      ;;
    roscore_wait:=*)
      _roscore_wait="${arg#*=}"
      ;;
    ros_master_uri:=*)
      _ros_master_uri="${arg#*=}"
      ;;
    ros_ip:=*)
      _ros_ip="${arg#*=}"
      ;;
    use_sim_time:=*)
      _use_sim_time="${arg#*=}"
      ;;
    *)
      merge_arg "$arg"
      ;;
  esac
done

synchronize_agent_number_with_robot_ids

source "${ROOT_DIR}/src/ros_motion_planning/scripts/env.sh"

export ROS_MASTER_URI="${_ros_master_uri}"
if [[ -n "${_ros_ip}" ]]; then
  export ROS_IP="${_ros_ip}"
fi

_cleanup() {
  if [[ -n "${HOST_STACK_PID}" ]] && kill -0 "${HOST_STACK_PID}" 2>/dev/null; then
    kill "${HOST_STACK_PID}" >/dev/null 2>&1 || true
    wait "${HOST_STACK_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ROSCORE_PID}" ]] && kill -0 "${ROSCORE_PID}" 2>/dev/null; then
    kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
    wait "${ROSCORE_PID}" 2>/dev/null || true
  fi
}

_wait_for_master() {
  python - <<PY2
import os
import sys
import time
import subprocess

deadline = time.time() + float(${_roscore_wait})
env = os.environ.copy()
while time.time() < deadline:
    proc = subprocess.run(['rosnode', 'list'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    if proc.returncode == 0:
        sys.exit(0)
    time.sleep(0.2)
sys.exit(1)
PY2
}

trap _cleanup EXIT INT TERM

if [[ "${_start_roscore}" == "true" ]]; then
  if rosnode list >/dev/null 2>&1; then
    echo "[start_host] ROS master already reachable at ${ROS_MASTER_URI}"
  else
    echo "[start_host] Starting roscore"
    roscore &
    ROSCORE_PID=$!
    if ! _wait_for_master; then
      echo "[start_host] Failed to reach ROS master after ${_roscore_wait}s" >&2
      exit 1
    fi
  fi
else
  if ! rosnode list >/dev/null 2>&1; then
    echo "[start_host] ROS master is not reachable at ${ROS_MASTER_URI}" >&2
    exit 1
  fi
fi

rosparam set /use_sim_time "${_use_sim_time}"
echo "[start_host] Set /use_sim_time=${_use_sim_time}"

echo "[start_host] Using config file: ${_config_file}"
printf '[start_host] Forward args:'
printf ' %q' "${host_args[@]}"
printf '\\n'

shape_assembly_args=(
  "launch_map_server:=${_launch_map_server}"
  "auto_start_gather:=${_auto_start_gather}"
  "start_gather_delay:=${_start_gather_delay}"
  "start_gather_wait_started:=${_start_gather_wait_started}"
  "start_gather_wait_connections:=${_start_gather_wait_connections}"
  "start_gather_repeat:=${_start_gather_repeat}"
  "start_gather_rate:=${_start_gather_rate}"
)
if [[ -n "${_map_file}" ]]; then
  shape_assembly_args+=("map_file:=${_map_file}")
fi
shape_assembly_args+=("${host_args[@]}")

bash "${ROOT_DIR}/src/ros_motion_planning/scripts/shape_assembly_real_robot.sh" "${shape_assembly_args[@]}" &
HOST_STACK_PID=$!
if [[ "${_auto_start_gather}" != "true" ]]; then
  echo "[start_host] Host stack started. Trigger gather manually with:"
  echo "[start_host]   rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'"
fi
wait "${HOST_STACK_PID}"
