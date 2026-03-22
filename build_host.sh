#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

source /opt/ros/noetic/setup.bash

PACKAGES=(
  formation_msgs
  formation_host
  move_base_client
  fm2_gather
  sim_env
)

WHITELIST="$(IFS=';'; echo "${PACKAGES[*]}")"

print_packages() {
  printf '%s\n' "${PACKAGES[@]}"
}

usage() {
  cat <<EOF
Usage: ./build_host.sh [--list] [catkin_make args...]

Options:
  --list    Print the host-side package whitelist and exit.

Default behavior:
  Run catkin_make with CATKIN_WHITELIST_PACKAGES set to the host-side packages.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--list" ]]; then
  print_packages
  exit 0
fi

SHAPE_DIR="$ROOT_DIR/src/ros_motion_planning/src/sim_env/shape_images"
if [[ ! -d "$SHAPE_DIR" ]]; then
  echo "[build_host] Warning: shape_images directory not found: $SHAPE_DIR" >&2
fi

echo "[build_host] Building packages: $WHITELIST"
catkin_make -DCATKIN_WHITELIST_PACKAGES="$WHITELIST" "$@"
