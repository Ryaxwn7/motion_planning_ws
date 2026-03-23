#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

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
  "use_center_as_goal:=false"
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

export ROS_MASTER_URI="${_ros_master_uri}"
if [[ -n "${_ros_ip}" ]]; then
  export ROS_IP="${_ros_ip}"
fi

echo "[start_robot] Using config file: ${_config_file}"
printf '[start_robot] Launch args:'
printf ' %q' "${robot_args[@]}"
printf '\\n'

exec roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch   "${robot_args[@]}"
