#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: env.sh must be sourced, not executed." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
