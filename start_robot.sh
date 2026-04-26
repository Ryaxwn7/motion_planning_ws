#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

_config_file="${ROOT_DIR}/config/robot_param.yaml"
_ros_master_uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
_ros_ip="${ROS_IP:-}"
_time_sync_enabled=true
_time_sync_server="192.168.1.104"
_config_temp_dir=""
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
  ${ROOT_DIR}/config/robot_param.yaml

Script args:
  config:=/abs/or/relative/path.yaml
  ros_master_uri:=http://<HOST_IP>:11311
  ros_ip:=<ROBOT_IP>
  time_sync_enabled:=true|false
  time_sync_server:=<HOST_IP>

All other args are forwarded to motion_navigate_multi4.launch.
Command-line args override config file values.
Both YAML and legacy shell `.conf` configs are supported.
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
  case "${_config_file}" in
    *.yaml|*.yml)
      # shellcheck disable=SC1090
      source <(python3 "${ROOT_DIR}/src/ros_motion_planning/scripts/startup_param_loader.py" --mode robot --config "$_config_file" --ws-root "${ROOT_DIR}")
      _config_temp_dir="${CONFIG_TEMP_DIR:-}"
      ;;
    *)
      # shellcheck disable=SC1090
      source "$_config_file"
      ;;
  esac
else
  echo "[start_robot] Warning: config file not found: $_config_file" >&2
fi

if [[ ${ROS_MASTER_URI_VALUE+x} ]]; then
  _ros_master_uri="$ROS_MASTER_URI_VALUE"
fi
if [[ ${ROS_IP_VALUE+x} ]]; then
  _ros_ip="$ROS_IP_VALUE"
fi
if [[ ${TIME_SYNC_ENABLED+x} ]]; then
  _time_sync_enabled="$TIME_SYNC_ENABLED"
fi
if [[ ${TIME_SYNC_SERVER+x} ]]; then
  _time_sync_server="$TIME_SYNC_SERVER"
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
    time_sync_enabled:=*)
      _time_sync_enabled="${arg#*=}"
      ;;
    time_sync_server:=*)
      _time_sync_server="${arg#*=}"
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

_cleanup() {
  if [[ -n "${_config_temp_dir}" && -d "${_config_temp_dir}" ]]; then
    rm -rf "${_config_temp_dir}" || true
  fi
}

trap _cleanup EXIT INT TERM

echo "[start_robot] Using config file: ${_config_file}"
printf '[start_robot] Launch args:'
printf ' %q' "${robot_args[@]}"
printf '\\n'
if [[ "${_time_sync_enabled}" == "true" && -n "${_time_sync_server}" ]]; then
  sudo /etc/init.d/ntp stop
  sudo ntpdate "${_time_sync_server}"
fi
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch "${robot_args[@]}"
