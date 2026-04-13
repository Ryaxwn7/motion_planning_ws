#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SHAPE_ASSEMBLY_USER_CONFIG_BASENAME="${SHAPE_ASSEMBLY_USER_CONFIG_BASENAME:-user_config_5x10_d2_amcl_simtest.yaml}"

exec "${SCRIPT_DIR}/shape_assembly_5x10_d2.sh" "$@"
