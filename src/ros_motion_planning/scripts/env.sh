#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: env.sh must be sourced, not executed." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_filter_colon_path_var() {
  local var_name="$1"
  local raw_value="${!var_name:-}"
  local blocked_prefix
  local entry
  local filtered=()
  local blocked=0
  local _path_entries=()

  [[ -z "${raw_value}" ]] && return 0

  IFS=':' read -r -a _path_entries <<< "${raw_value}"
  for entry in "${_path_entries[@]}"; do
    blocked=0
    for blocked_prefix in "${_ROS_ENV_BLOCKED_PREFIXES[@]}"; do
      if [[ "${entry}" == "${blocked_prefix}" || "${entry}" == "${blocked_prefix}/"* ]]; then
        blocked=1
        _ROS_ENV_REMOVED=1
        break
      fi
    done
    if [[ "${blocked}" -eq 0 ]]; then
      filtered+=("${entry}")
    fi
  done

  if [[ "${#filtered[@]}" -gt 0 ]]; then
    local joined
    printf -v joined '%s:' "${filtered[@]}"
    export "${var_name}=${joined%:}"
  else
    unset "${var_name}"
  fi
}

_clean_inherited_ros_overlay() {
  local default_blocked="/home/yxw/new_ws"
  local configured="${ROS_ENV_BLOCKED_PREFIXES:-${default_blocked}}"
  local prefix
  local _configured_prefixes=()
  _ROS_ENV_BLOCKED_PREFIXES=()
  _ROS_ENV_REMOVED=0

  IFS=':' read -r -a _configured_prefixes <<< "${configured}"
  for prefix in "${_configured_prefixes[@]}"; do
    [[ -n "${prefix}" ]] && _ROS_ENV_BLOCKED_PREFIXES+=("${prefix}")
  done

  [[ "${#_ROS_ENV_BLOCKED_PREFIXES[@]}" -eq 0 ]] && return 0

  _filter_colon_path_var CMAKE_PREFIX_PATH
  _filter_colon_path_var ROS_PACKAGE_PATH
  _filter_colon_path_var PYTHONPATH
  _filter_colon_path_var ROSLISP_PACKAGE_DIRECTORIES
  _filter_colon_path_var LD_LIBRARY_PATH
  _filter_colon_path_var PKG_CONFIG_PATH

  if [[ "${_ROS_ENV_REMOVED}" -eq 1 && "${ROS_ENV_QUIET:-0}" != "1" ]]; then
    echo "[env.sh] Removed inherited ROS overlay entries under: ${configured}" >&2
  fi

  unset _ROS_ENV_BLOCKED_PREFIXES _ROS_ENV_REMOVED
}

_clean_inherited_ros_overlay

SEARCH_DIR="${SCRIPT_DIR}"
best_devel=""
best_install=""
for _ in 1 2 3 4 5 6 7 8; do
  if [[ -f "${SEARCH_DIR}/devel/setup.bash" ]]; then
    best_devel="${SEARCH_DIR}/devel/setup.bash"
  fi
  if [[ -f "${SEARCH_DIR}/install/setup.bash" ]]; then
    best_install="${SEARCH_DIR}/install/setup.bash"
  fi
  SEARCH_DIR="$(cd "${SEARCH_DIR}/.." && pwd)"
done

if [[ -n "${best_devel}" ]]; then
  # shellcheck disable=SC1090
  source "${best_devel}"
  return 0
fi
if [[ -n "${best_install}" ]]; then
  # shellcheck disable=SC1090
  source "${best_install}"
  return 0
fi
if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/noetic/setup.bash"
else
  echo "ERROR: ROS setup.bash not found. Build the workspace or source ROS first." >&2
  return 1
fi
