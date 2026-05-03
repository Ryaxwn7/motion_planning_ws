#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

source "${SCRIPT_DIR}/env.sh"

LOCK_FILE="/tmp/shape_assembly_real_robot.lock"
MAP_PID=""
HOST_PID=""

_launch_map_server=true
_auto_start_gather=false
_start_gather_delay=2.0
_start_gather_wait_started=5.0
_start_gather_wait_connections=2.0
_start_gather_repeat=3
_start_gather_rate=5.0
_map_file="$(rospack find turn_on_wheeltec_robot)/map/exp_d2.yaml"
launch_args=()

_acquire_lock() {
  if [[ -f "${LOCK_FILE}" ]]; then
    old_pid="$(cat "${LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
      echo "shape_assembly_real_robot.sh is already running (pid=${old_pid})."
      exit 1
    fi
    rm -f "${LOCK_FILE}" || true
  fi
  echo "$$" > "${LOCK_FILE}"
}

_cleanup() {
  if [[ -n "${HOST_PID}" ]] && kill -0 "${HOST_PID}" 2>/dev/null; then
    kill "${HOST_PID}" >/dev/null 2>&1 || true
    wait "${HOST_PID}" 2>/dev/null || true
  fi
  if [[ -n "${MAP_PID}" ]] && kill -0 "${MAP_PID}" 2>/dev/null; then
    kill "${MAP_PID}" >/dev/null 2>&1 || true
    wait "${MAP_PID}" 2>/dev/null || true
  fi
  rm -f "${LOCK_FILE}" || true
}

for arg in "$@"; do
  case "${arg}" in
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
    *)
      launch_args+=("${arg}")
      ;;
  esac
done

_acquire_lock
trap _cleanup EXIT INT TERM

if [[ "${_launch_map_server}" == "true" ]]; then
  echo "[shape_assembly_real_robot] Starting map_server with ${_map_file}"
  rosrun map_server map_server "${_map_file}" &
  MAP_PID=$!
  sleep 1
fi

echo "[shape_assembly_real_robot] Starting host formation stack"
roslaunch turn_on_wheeltec_robot shape_assembly_host.launch "${launch_args[@]}" &
HOST_PID=$!
if [[ "${_auto_start_gather}" != "true" ]]; then
  echo "[shape_assembly_real_robot] Host is waiting for manual gather trigger:"
  echo "[shape_assembly_real_robot]   rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'"
fi

if [[ "${_auto_start_gather}" == "true" ]]; then
  echo "[shape_assembly_real_robot] Waiting ${_start_gather_delay}s before triggering fm2_gather"
  sleep "${_start_gather_delay}"
  rosrun move_base_client start_gather.py     --wait-started "${_start_gather_wait_started}"     --wait-connections "${_start_gather_wait_connections}"     --repeat "${_start_gather_repeat}"     --rate "${_start_gather_rate}"
fi

wait "${HOST_PID}"
