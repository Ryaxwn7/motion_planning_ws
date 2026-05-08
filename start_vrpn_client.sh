#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

usage() {
  cat <<EOF
Usage: ./start_vrpn_client.sh [args] [extra vrpn launch args]

Starts vrpn_client_ros and publishes the world/map transform.

Args:
  server:=192.168.1.117
  parent_frame:=world
  child_frame:=map
  x:=0.0
  y:=0.0
  z:=0.0
  yaw:=0.0
  pitch:=0.0
  roll:=0.0
  period_ms:=100
  tf_backend:=ui      # ui opens a live adjust window; live reads ROS params; tf2 publishes /tf_static; tf publishes repeated /tf
  tf_rate:=20.0

Example:
  ./start_vrpn_client.sh server:=192.168.1.117
  ./start_vrpn_client.sh server:=192.168.1.117 tf_backend:=ui
  ./start_vrpn_client.sh tf_backend:=live
  ./start_vrpn_client.sh server:=192.168.1.117 parent_frame:=world child_frame:=map
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --help|-h)
      usage
      exit 0
      ;;
  esac
done

server="192.168.1.102"
parent_frame="world"
child_frame="map"
x="0.0"
y="0.0"
z="0.0"
yaw="0.0"
pitch="0.0"
roll="0.0"
period_ms="100"
tf_backend="ui"
tf_rate="20.0"
vrpn_args=()

for arg in "$@"; do
  case "${arg}" in
    server:=*)
      server="${arg#*=}"
      ;;
    parent_frame:=*)
      parent_frame="${arg#*=}"
      ;;
    child_frame:=*)
      child_frame="${arg#*=}"
      ;;
    x:=*)
      x="${arg#*=}"
      ;;
    y:=*)
      y="${arg#*=}"
      ;;
    z:=*)
      z="${arg#*=}"
      ;;
    yaw:=*)
      yaw="${arg#*=}"
      ;;
    pitch:=*)
      pitch="${arg#*=}"
      ;;
    roll:=*)
      roll="${arg#*=}"
      ;;
    period_ms:=*)
      period_ms="${arg#*=}"
      ;;
    tf_backend:=*)
      tf_backend="${arg#*=}"
      ;;
    tf_rate:=*)
      tf_rate="${arg#*=}"
      ;;
    *)
      vrpn_args+=("${arg}")
      ;;
  esac
done

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

VRPN_PID=""
TF_PID=""

cleanup() {
  if [[ -n "${VRPN_PID}" ]] && kill -0 "${VRPN_PID}" 2>/dev/null; then
    kill "${VRPN_PID}" >/dev/null 2>&1 || true
    wait "${VRPN_PID}" 2>/dev/null || true
  fi
  if [[ -n "${TF_PID}" ]] && kill -0 "${TF_PID}" 2>/dev/null; then
    kill "${TF_PID}" >/dev/null 2>&1 || true
    wait "${TF_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "[start_vrpn_client] Starting vrpn_client_ros sample.launch server:=${server}"
roslaunch vrpn_client_ros sample.launch "server:=${server}" "${vrpn_args[@]}" &
VRPN_PID=$!

echo "[start_vrpn_client] Publishing TF ${parent_frame} -> ${child_frame} with backend=${tf_backend}"
echo "[start_vrpn_client] transform xyz=(${x}, ${y}, ${z}) rpy=(${roll}, ${pitch}, ${yaw}) period_ms=${period_ms}"
if [[ "${tf_backend}" == "ui" ]]; then
  python3 "${ROOT_DIR}/scripts/tf_adjust_ui.py" \
    __name:=tf_adjust_ui \
    _parent_frame:="${parent_frame}" \
    _child_frame:="${child_frame}" \
    _x:="${x}" \
    _y:="${y}" \
    _z:="${z}" \
    _roll:="${roll}" \
    _pitch:="${pitch}" \
    _yaw:="${yaw}" \
    _rate:="${tf_rate}" &
elif [[ "${tf_backend}" == "live" ]]; then
  rosparam set /live_map_world_tf/parent_frame "${parent_frame}"
  rosparam set /live_map_world_tf/child_frame "${child_frame}"
  rosparam set /live_map_world_tf/x "${x}"
  rosparam set /live_map_world_tf/y "${y}"
  rosparam set /live_map_world_tf/z "${z}"
  rosparam set /live_map_world_tf/roll "${roll}"
  rosparam set /live_map_world_tf/pitch "${pitch}"
  rosparam set /live_map_world_tf/yaw "${yaw}"
  rosparam set /live_map_world_tf/rate "${tf_rate}"
  python3 "${ROOT_DIR}/scripts/live_tf_publisher.py" __name:=live_map_world_tf &
elif [[ "${tf_backend}" == "tf2" ]]; then
  rosrun tf2_ros static_transform_publisher \
    "${x}" "${y}" "${z}" \
    "${yaw}" "${pitch}" "${roll}" \
    "${parent_frame}" "${child_frame}" &
else
  rosrun tf static_transform_publisher \
    "${x}" "${y}" "${z}" \
    "${yaw}" "${pitch}" "${roll}" \
    "${parent_frame}" "${child_frame}" "${period_ms}" &
fi
TF_PID=$!

wait "${VRPN_PID}"
