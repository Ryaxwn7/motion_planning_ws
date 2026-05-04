#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

source "${ROOT_DIR}/src/ros_motion_planning/scripts/env.sh"

_config_file="${ROOT_DIR}/config/robot_start.conf"
_ros_master_uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
_ros_ip="${ROS_IP:-}"
robot_args=(
  "map_started:=true"
  "agent_number:=4"
  "agent_id:=1"
  "global_planner:=fm2"
  "local_planner:=my"
  "robot:=mini_mec"
  "enable_shape_assembly:=true"
  "shape_source:=mat"
  "shape_type:=rectangle"
  "use_center_as_goal:=true"
)

usage() {
  cat <<EOF
Usage: ./start_robot.sh [script args] [motion_navigate_multi4 args]

Default config file:
  ${ROOT_DIR}/config/robot_start.conf

Script args:
  config:=/abs/or/relative/path.conf
  ros_master_uri:=http://<HOST_IP>:11311
  ros_ip:=<ROBOT_IP>

All other args are forwarded to motion_navigate_multi4.launch.
Command-line args override config file values.
EOF
}

merge_arg() {
  local candidate="$1"
  if [[ "$candidate" != *":="* ]]; then
    robot_args+=("$candidate")
    return
  fi
  local key="${candidate%%:=*}"
  local i
  for i in "${!robot_args[@]}"; do
    if [[ "${robot_args[$i]}" == "${key}:="* ]]; then
      robot_args[$i]="$candidate"
      return
    fi
  done
  robot_args+=("$candidate")
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
  for arg in "${robot_args[@]}"; do
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
  echo "[start_robot] Warning: config file not found: $_config_file" >&2
fi

if [[ ${ROS_MASTER_URI_VALUE+x} ]]; then
  _ros_master_uri="$ROS_MASTER_URI_VALUE"
fi
if [[ ${ROS_IP_VALUE+x} ]]; then
  _ros_ip="$ROS_IP_VALUE"
fi
if declare -p ROBOT_LAUNCH_ARGS >/dev/null 2>&1; then
  robot_args=("${ROBOT_LAUNCH_ARGS[@]}")
fi

for arg in "$@"; do
  case "$arg" in
    --help|-h|config:=*|--config=*)
      ;;
    ros_master_uri:=*)
      _ros_master_uri="${arg#*=}"
      ;;
    ros_ip:=*)
      _ros_ip="${arg#*=}"
      ;;
    *)
      merge_arg "$arg"
      ;;
  esac
done

synchronize_agent_number_with_robot_ids

export ROS_MASTER_URI="${_ros_master_uri}"
if [[ -n "${_ros_ip}" ]]; then
  export ROS_IP="${_ros_ip}"
fi

# --- Auto-sync shared params from host param server ---
# Pull host-side config from global ROS params if master is reachable.
# These override local defaults only when the host has already set them.
_pull_host_param() {
  local param_name="$1"
  local arg_key="$2"
  local value
  if value="$(rosparam get "${param_name}" 2>/dev/null)"; then
    if [[ -n "${value}" ]]; then
      merge_arg "${arg_key}:=${value}"
      return 0
    fi
  fi
  return 1
}

if rosnode list >/dev/null 2>&1; then
  _pulled=0
  _pull_host_param /robot_ids robot_ids && _pulled=1
  _pull_host_param /shape_type shape_type && _pulled=1
  _pull_host_param /shape_scale shape_scale && _pulled=1
  _pull_host_param /shape_source shape_source && _pulled=1
  if [[ "${_pulled}" -eq 1 ]]; then
    synchronize_agent_number_with_robot_ids
    echo "[start_robot] Synced params from host param server"
  fi
else
  echo "[start_robot] ROS master not reachable, using local config defaults"
fi
# --- end auto-sync ---

echo "[start_robot] Using config file: ${_config_file}"
printf '[start_robot] Launch args:'
printf ' %q' "${robot_args[@]}"
printf '\\n'
sudo /etc/init.d/ntp stop
sudo ntpdate 192.168.1.104
exec roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch   "${robot_args[@]}"
