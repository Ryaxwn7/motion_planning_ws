#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

source /opt/ros/noetic/setup.bash

PACKAGES=(
  formation_msgs
  formation_robot
  sim_env
  utils
  curve_generation
  global_planner
  voronoi_layer
  graph_planner
  my_planner
  my_ware
  lslidar_msgs
  lslidar_driver
  turn_on_wheeltec_robot
)

WHITELIST="$(IFS=';'; echo "${PACKAGES[*]}")"

print_packages() {
  printf '%s\n' "${PACKAGES[@]}"
}

usage() {
  cat <<EOF
Usage: ./build_robot.sh [--list] [catkin_make args...]

Options:
  --list    Print the robot-side package whitelist and exit.

Default behavior:
  Run catkin_make with CATKIN_WHITELIST_PACKAGES set to the robot-side packages.
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
  echo "[build_robot] Warning: shape_images directory not found: $SHAPE_DIR" >&2
fi

if ! rospack find serial >/dev/null 2>&1; then
  echo "[build_robot] Warning: ROS package 'serial' not found. turn_on_wheeltec_robot may fail to build until the dependency is installed." >&2
fi

echo "[build_robot] Building packages: $WHITELIST"
catkin_make -DCATKIN_WHITELIST_PACKAGES="$WHITELIST" "$@"
