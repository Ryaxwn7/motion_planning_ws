#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

source "${ROOT_DIR}/src/ros_motion_planning/scripts/env.sh"

exec python3 "${ROOT_DIR}/src/formation_host/scripts/host_control_ui.py" "$@"
