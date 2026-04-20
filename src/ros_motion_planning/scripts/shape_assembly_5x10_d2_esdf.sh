#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

args=("$@")
has_backend=0
for arg in "${args[@]}"; do
  if [[ "${arg}" == fm2_distance_field_backend:=* ]]; then
    has_backend=1
    break
  fi
done

if [[ "${has_backend}" -eq 0 ]]; then
  args=("fm2_distance_field_backend:=1" "${args[@]}")
fi

exec "${SCRIPT_DIR}/shape_assembly_5x10_d2.sh" "${args[@]}"
