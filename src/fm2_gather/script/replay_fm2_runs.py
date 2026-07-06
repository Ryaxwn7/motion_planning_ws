#!/usr/bin/env python3

import argparse
import csv
import datetime as dt
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = WORKSPACE / "src/fm2_gather/config/gather_param_real_simaligned.yaml"
DEFAULT_OUTPUT_ROOT = WORKSPACE / "okbags/replay_results"

RUNS = {
    "run_20260508_184457_0004_gather_fastest": {
        "bag": "okbags/2026-05-08-18-44-5-sphere-fast-ok2.bag",
        "mission": "fastest",
    },
    "run_20260508_184459_0005_gather_fastest": {
        "bag": "okbags/2026-05-08-18-44-5-sphere-fast-ok2.bag",
        "mission": "fastest",
    },
    "run_20260508_190057_0011_gather_energy": {
        "bag": "okbags/2026-05-08-19-00-37-energy-ok.bag",
        "mission": "energy",
    },
    "run_20260508_192912_0002_gather_space": {
        "bag": "okbags/2026-05-08-19-28-53-space-ok.bag",
        "mission": "space",
    },
    "run_20260508_192913_0003_gather_space": {
        "bag": "okbags/2026-05-08-19-28-53-space-ok.bag",
        "mission": "space",
    },
    "run_20260508_220944_0001_gather_fastest": {
        "bag": "okbags/2026-05-08-22-09-41-cut.bag",
        "mission": "fastest",
    },
    "run_20260508_220947_0002_gather_fastest": {
        "bag": "okbags/2026-05-08-22-09-41-cut.bag",
        "mission": "fastest",
    },
    "run_20260508_221002_0003_gather_fastest": {
        "bag": "okbags/2026-05-08-22-09-41-cut.bag",
        "mission": "fastest",
    },
    "run_20260508_221004_0004_gather_fastest": {
        "bag": "okbags/2026-05-08-22-09-41-cut.bag",
        "mission": "fastest",
    },
}


ROBOT_IDS = [3, 4, 5, 6]


def shell_env():
    env = os.environ.copy()
    setup_files = [
        Path("/opt/ros/noetic/setup.bash"),
        WORKSPACE / "devel/setup.bash",
    ]
    source_parts = [f"source {p}" for p in setup_files if p.exists()]
    env["_FM2_REPLAY_SOURCE"] = " && ".join(source_parts)
    return env


def ros_cmd(command):
    source = os.environ.get("_FM2_REPLAY_SOURCE", "")
    if source:
        return ["bash", "-lc", f"{source} && exec {command}"]
    return ["bash", "-lc", f"exec {command}"]


def run_checked(command, env, log_file=None):
    stdout = subprocess.PIPE if log_file is None else log_file
    stderr = subprocess.STDOUT
    proc = subprocess.run(ros_cmd(command), cwd=str(WORKSPACE), env=env, stdout=stdout, stderr=stderr, text=True)
    if proc.returncode != 0:
        output = proc.stdout if proc.stdout else ""
        raise RuntimeError(f"command failed ({proc.returncode}): {command}\n{output}")
    return proc.stdout or ""


def master_running(env):
    try:
        subprocess.run(
            ros_cmd("rosnode list"),
            cwd=str(WORKSPACE),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
            check=True,
        )
        return True
    except (subprocess.SubprocessError, OSError):
        return False


def start_roscore(env, log_path):
    log = log_path.open("w")
    proc = subprocess.Popen(ros_cmd("roscore"), cwd=str(WORKSPACE), env=env, stdout=log, stderr=subprocess.STDOUT)
    for _ in range(60):
        if proc.poll() is not None:
            raise RuntimeError(f"roscore exited early, see {log_path}")
        if master_running(env):
            return proc, log
        time.sleep(0.5)
    raise RuntimeError("timed out waiting for roscore")


def terminate_process(proc, timeout=8.0):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def input_topics():
    topics = [
        "/tf",
        "/tf_static",
        "/map",
        "/combined_map",
        "/gather_signal",
    ]
    for rid in ROBOT_IDS:
        base = f"/robot{rid}"
        topics.extend(
            [
                f"{base}/odom",
                f"{base}/robot_pose_ekf/odom_combined",
                f"{base}/move_base/local_costmap/costmap",
                f"{base}/move_base/global_costmap/costmap",
            ]
        )
    return topics


def set_params(env, config_path, mission):
    run_checked(f"rosparam load {config_path}", env)
    overrides = {
        "/use_sim_time": "true",
        "auto_detect_robots": "false",
        "num_robots": "4",
        "robot_namespace_prefix": "robot",
        "robot_detect_topic_suffix": "/odom",
        "robot_odom_topic_suffix": "/odom",
        "mission": mission,
        "publish_goal": "false",
        "use_move_base_controllers": "false",
        "save_data": "true",
        "debug_on": "false",
        "plot_on": "false",
        "publish_preview_paths": "false",
    }
    for key, value in overrides.items():
        run_checked(f"rosparam set {key} {value}", env)
    run_checked("rosparam set robot_ids '[3, 4, 5, 6]'", env)
    run_checked("rosparam set robot_weights '[1, 1, 1, 1]'", env)


def start_gather_node(env, output_dir, log_path):
    node_env = env.copy()
    node_env["FM2_GATHER_DEBUG_DIR"] = str(output_dir)
    log = log_path.open("w")
    proc = subprocess.Popen(
        ros_cmd("rosrun fm2_gather fm2_gather_node"),
        cwd=str(WORKSPACE),
        env=node_env,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    time.sleep(2.0)
    if proc.poll() is not None:
        raise RuntimeError(f"fm2_gather_node exited early, see {log_path}")
    return proc, log


def play_bag(env, bag_path, rate, log_path):
    topics = " ".join(input_topics())
    command = f"rosbag play --clock --delay=2.0 --rate={rate} {bag_path} --topics {topics}"
    log = log_path.open("w")
    proc = subprocess.Popen(ros_cmd(command), cwd=str(WORKSPACE), env=env, stdout=log, stderr=subprocess.STDOUT)
    return proc, log


def parse_summary(path):
    values = {}
    if not path.exists():
        return values
    with path.open() as f:
        for line in f:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def parse_compute_times(log_path):
    times = []
    if not log_path.exists():
        return times
    pattern = re.compile(r"Gather Compute time:\s+(\d+)\s+ms")
    with log_path.open(errors="ignore") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                times.append(int(match.group(1)))
    return times


def collect_results(output_dir, node_log, source_bag, expected_runs):
    run_dirs = sorted(p for p in output_dir.iterdir() if p.is_dir() and p.name.startswith("run_"))
    compute_times = parse_compute_times(node_log)
    rows = []
    for idx, run_dir in enumerate(run_dirs):
        summary = parse_summary(run_dir / "summary.txt")
        expected = expected_runs[idx] if idx < len(expected_runs) else ""
        rows.append(
            {
                "expected_run": expected,
                "replay_run": run_dir.name,
                "bag": str(source_bag),
                "compute_time_ms": compute_times[idx] if idx < len(compute_times) else "",
                "mission": summary.get("mission", ""),
                "min_idx": summary.get("min_idx", ""),
                "estimated_gather_cost": summary.get("estimated_gather_cost", ""),
                "center_grid": summary.get("center_grid", ""),
                "center_world": summary.get("center_world", ""),
                "summary_path": str(run_dir / "summary.txt"),
            }
        )
    return rows


def grouped_runs(selected):
    groups = []
    for run_name in selected:
        spec = RUNS[run_name]
        bag = str(WORKSPACE / spec["bag"])
        mission = spec["mission"]
        for group in groups:
            if group["bag"] == bag and group["mission"] == mission:
                group["runs"].append(run_name)
                break
        else:
            groups.append({"bag": bag, "mission": mission, "runs": [run_name]})
    return groups


def write_csv(rows, output_root):
    csv_path = output_root / "replay_summary.csv"
    fieldnames = [
        "expected_run",
        "replay_run",
        "bag",
        "compute_time_ms",
        "mission",
        "min_idx",
        "estimated_gather_cost",
        "center_grid",
        "center_world",
        "summary_path",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def main():
    parser = argparse.ArgumentParser(description="Replay okbags and reproduce fm2_gather run results.")
    parser.add_argument("runs", nargs="*", help="Run directory names to reproduce.")
    parser.add_argument("--all", action="store_true", help="Reproduce all known okbags experiment runs.")
    parser.add_argument("--rate", type=float, default=1.0, help="rosbag play rate. Use 1.0 for faithful timing.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="fm2_gather parameter YAML.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Directory for replay outputs.")
    parser.add_argument("--keep-roscore", action="store_true", help="Do not stop roscore started by this script.")
    args = parser.parse_args()

    if args.all:
        selected = list(RUNS.keys())
    else:
        selected = args.runs
    if not selected:
        raise SystemExit("select at least one run or pass --all")
    unknown = [name for name in selected if name not in RUNS]
    if unknown:
        raise SystemExit("unknown run(s): " + ", ".join(unknown))

    env = shell_env()
    os.environ["_FM2_REPLAY_SOURCE"] = env.get("_FM2_REPLAY_SOURCE", "")

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(args.output_root).expanduser().resolve() / stamp
    output_root.mkdir(parents=True, exist_ok=True)

    roscore_proc = None
    roscore_log = None
    started_roscore = False
    if not master_running(env):
        roscore_proc, roscore_log = start_roscore(env, output_root / "roscore.log")
        started_roscore = True

    all_rows = []
    try:
        for group_index, group in enumerate(grouped_runs(selected), 1):
            group_dir = output_root / f"group_{group_index:02d}_{Path(group['bag']).stem}_{group['mission']}"
            group_dir.mkdir(parents=True, exist_ok=True)
            set_params(env, Path(args.config).expanduser().resolve(), group["mission"])

            node_proc = node_log = play_proc = play_log = None
            try:
                node_proc, node_log = start_gather_node(env, group_dir, group_dir / "fm2_gather_node.log")
                play_proc, play_log = play_bag(env, group["bag"], args.rate, group_dir / "rosbag_play.log")
                play_rc = play_proc.wait()
                if play_rc != 0:
                    raise RuntimeError(f"rosbag play failed with code {play_rc}, see {group_dir / 'rosbag_play.log'}")
                time.sleep(2.0)
            finally:
                terminate_process(node_proc)
                if node_log:
                    node_log.close()
                if play_log:
                    play_log.close()

            rows = collect_results(
                group_dir,
                group_dir / "fm2_gather_node.log",
                group["bag"],
                group["runs"],
            )
            all_rows.extend(rows)
            print(f"{group['bag']} -> {len(rows)} reproduced run(s)")

        csv_path = write_csv(all_rows, output_root)
        print(f"summary: {csv_path}")
        for row in all_rows:
            print(
                "{expected_run} => {replay_run}, time={compute_time_ms} ms, "
                "center={center_world}, cost={estimated_gather_cost}".format(**row)
            )
    finally:
        if started_roscore and not args.keep_roscore:
            terminate_process(roscore_proc)
        if roscore_log:
            roscore_log.close()


if __name__ == "__main__":
    main()
