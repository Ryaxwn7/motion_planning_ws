#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

DURATION="${BENCHMARK_DURATION_SEC:-30}"
ROBOT_COUNT="${BENCHMARK_ROBOT_COUNT:-4}"
ROBOT_PREFIX="${BENCHMARK_ROBOT_PREFIX:-robot}"
USER_CONFIG_BASENAME="${BENCHMARK_USER_CONFIG_BASENAME:-user_config_5x10_d2_benchmark_simtest.yaml}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${BENCHMARK_OUTPUT_DIR:-${SCRIPT_DIR}/../../benchmark-results/fm2_backend_benchmark_${TIMESTAMP}}"
COLLECTOR="${SCRIPT_DIR}/fm2_benchmark_collect.py"
ESDF_SCRIPT="${SCRIPT_DIR}/shape_assembly_5x10_d2_esdf.sh"
FMM_SCRIPT="${SCRIPT_DIR}/shape_assembly_5x10_d2_fmm.sh"

mkdir -p "${OUT_DIR}"

wait_for_master() {
  local timeout_sec="${1:-60}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if rosnode list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_node() {
  local node_name="$1"
  local timeout_sec="${2:-120}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if rosnode list 2>/dev/null | grep -Fx "${node_name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

cleanup_case() {
  local launcher_pid="${1:-}"
  local collector_pid="${2:-}"
  if [[ -n "${collector_pid}" ]]; then
    kill "${collector_pid}" >/dev/null 2>&1 || true
    wait "${collector_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${launcher_pid}" ]]; then
    kill -TERM -- "-${launcher_pid}" >/dev/null 2>&1 || kill "${launcher_pid}" >/dev/null 2>&1 || true
    sleep 2
    pkill -TERM -g "${launcher_pid}" >/dev/null 2>&1 || true
    wait "${launcher_pid}" >/dev/null 2>&1 || true
  fi
}

run_case() {
  local backend="$1"
  local script_path="$2"
  local run_dir="${OUT_DIR}/${backend}"
  mkdir -p "${run_dir}"

  local raw_csv="${run_dir}/raw.csv"
  local summary_csv="${run_dir}/summary.csv"
  local launcher_log="${run_dir}/launcher.log"

  local launcher_pid=""
  local collector_pid=""

  echo "[benchmark] starting ${backend} run"
  env SHAPE_ASSEMBLY_USER_CONFIG_BASENAME="${USER_CONFIG_BASENAME}" \
    setsid "${script_path}" >"${launcher_log}" 2>&1 &
  launcher_pid=$!

  if ! wait_for_master 60; then
    echo "[benchmark] ROS master did not become ready for ${backend}" >&2
    cleanup_case "${launcher_pid}" "${collector_pid}"
    return 1
  fi

  if ! wait_for_node "/robot1/move_base" 180; then
    echo "[benchmark] /robot1/move_base did not come up for ${backend}" >&2
    cleanup_case "${launcher_pid}" "${collector_pid}"
    return 1
  fi
  if ! wait_for_node "/fm2_gather_node" 180; then
    echo "[benchmark] /fm2_gather_node did not come up for ${backend}" >&2
    cleanup_case "${launcher_pid}" "${collector_pid}"
    return 1
  fi

  python3 "${COLLECTOR}" \
    --backend "${backend}" \
    --duration "${DURATION}" \
    --robot-prefix "${ROBOT_PREFIX}" \
    --robot-count "${ROBOT_COUNT}" \
    --raw-csv "${raw_csv}" \
    --summary-csv "${summary_csv}" &
  collector_pid=$!

  sleep 3
  rosrun move_base_client start_gather.py --wait-connections 3 --wait-started 10 --repeat 5 --rate 2 >/dev/null 2>&1 || true

  wait "${collector_pid}"
  collector_pid=""
  cleanup_case "${launcher_pid}" "${collector_pid}"
  sleep 5
}

run_case esdf "${ESDF_SCRIPT}"
run_case fmm "${FMM_SCRIPT}"

python3 - "${OUT_DIR}" <<'PY'
import csv
import math
import sys
from pathlib import Path

out_dir = Path(sys.argv[1])
summary_data = {}
for backend in ("esdf", "fmm"):
    path = out_dir / backend / "summary.csv"
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["scope"] != "ALL":
                continue
            rows.append(row)
    summary_data[backend] = {row["metric"]: row for row in rows}

metrics = [
    ("planning_time", "Planning Time"),
    ("distance_field_update_time", "Distance Field Update Time"),
    ("velocity_map_time", "Velocity Map Time"),
]

comparison_csv = out_dir / "comparison.csv"
comparison_md = out_dir / "comparison.md"

with comparison_csv.open("w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["metric", "esdf_count", "esdf_mean_ms", "fmm_count", "fmm_mean_ms", "delta_ms", "delta_percent"])
    for metric, _ in metrics:
        esdf = summary_data.get("esdf", {}).get(metric)
        fmm = summary_data.get("fmm", {}).get(metric)
        if not esdf or not fmm:
            writer.writerow([metric, "", "", "", "", "", ""])
            continue
        esdf_mean = float(esdf["mean_ms"])
        fmm_mean = float(fmm["mean_ms"])
        delta_ms = esdf_mean - fmm_mean
        delta_pct = (delta_ms / fmm_mean * 100.0) if abs(fmm_mean) > 1e-9 else math.nan
        writer.writerow([metric, esdf["count"], f"{esdf_mean:.6f}", fmm["count"], f"{fmm_mean:.6f}", f"{delta_ms:.6f}", f"{delta_pct:.2f}"])

lines = [
    "# FM2 Backend Benchmark",
    "",
    f"Output directory: `{out_dir}`",
    "",
    "| Metric | ESDF count | ESDF mean ms | FMM count | FMM mean ms | Delta ms | Delta % |",
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
]
for metric, label in metrics:
    esdf = summary_data.get("esdf", {}).get(metric)
    fmm = summary_data.get("fmm", {}).get(metric)
    if not esdf or not fmm:
        lines.append(f"| {label} | - | - | - | - | - | - |")
        continue
    esdf_mean = float(esdf["mean_ms"])
    fmm_mean = float(fmm["mean_ms"])
    delta_ms = esdf_mean - fmm_mean
    delta_pct = (delta_ms / fmm_mean * 100.0) if abs(fmm_mean) > 1e-9 else math.nan
    lines.append(
        f"| {label} | {esdf['count']} | {esdf_mean:.3f} | {fmm['count']} | {fmm_mean:.3f} | {delta_ms:.3f} | {delta_pct:.2f}% |"
    )

comparison_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(comparison_md)
PY

echo "[benchmark] results written to ${OUT_DIR}"
