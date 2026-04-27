#!/usr/bin/env python3

import json
import os
import queue
import re
import signal
import subprocess
import threading
import time
import shlex
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rosgraph
import rospy
import tkinter as tk
from actionlib_msgs.msg import GoalID
from formation_msgs.msg import RobotFormationStatus, ShapeTask
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8
from tkinter import filedialog, messagebox, scrolledtext, ttk

try:
    from dynamic_reconfigure.client import Client as DynamicReconfigureClient
except Exception:
    DynamicReconfigureClient = None


PHASE_LABELS = {
    RobotFormationStatus.PHASE_IDLE: "Idle",
    RobotFormationStatus.PHASE_NAVIGATE: "Navigate",
    RobotFormationStatus.PHASE_STAGING: "Staging",
    RobotFormationStatus.PHASE_SHAPE_ACTIVE: "ShapeActive",
    RobotFormationStatus.PHASE_CONVERGED: "Converged",
    RobotFormationStatus.PHASE_ERROR: "Error",
}

SHAPE_CHOICES = [
    "rectangle",
    "ring",
    "triangle",
    "line",
    "sphere",
    "letter_o",
    "letter_r",
    "letter_b",
    "starfish",
    "snowflake",
]

CONNECTION_TIMEOUT_SEC = 3.0
STALE_TIMEOUT_SEC = 10.0

SCRIPT_ARG_DEFS: List[Tuple[str, str, str]] = [
    ("start_roscore", "START_ROSCORE", "false"),
    ("launch_map_server", "LAUNCH_MAP_SERVER", "true"),
    ("auto_start_gather", "AUTO_START_GATHER", "false"),
    ("roscore_wait", "ROSCORE_WAIT", "8.0"),
    ("ros_master_uri", "ROS_MASTER_URI_VALUE", ""),
    ("ros_ip", "ROS_IP_VALUE", ""),
    ("map_file", "MAP_FILE_VALUE", ""),
    ("start_gather_delay", "START_GATHER_DELAY", "2.0"),
    ("start_gather_wait_started", "START_GATHER_WAIT_STARTED", "5.0"),
    ("start_gather_wait_connections", "START_GATHER_WAIT_CONNECTIONS", "2.0"),
    ("start_gather_repeat", "START_GATHER_REPEAT", "3"),
    ("start_gather_rate", "START_GATHER_RATE", "5.0"),
]

SCRIPT_ARG_KEYS = [item[0] for item in SCRIPT_ARG_DEFS]
SCRIPT_ARG_VAR_MAP = {item[1]: item[0] for item in SCRIPT_ARG_DEFS}
SCRIPT_ARG_DEFAULTS = {item[0]: item[2] for item in SCRIPT_ARG_DEFS}
COMMON_HOST_ARG_KEYS = ("agent_number", "robot_ids", "shape_type", "shape_scale")


def parse_robot_ids(text: str) -> List[int]:
    values = []
    for item in str(text).replace("[", "").replace("]", "").split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value <= 0:
            raise ValueError("robot id must be positive")
        values.append(value)
    if not values:
        raise ValueError("robot ids cannot be empty")
    return values


def robot_ids_arg(robot_ids: List[int]) -> str:
    return "robot_ids:=[{}]".format(",".join(str(v) for v in robot_ids))


def launch_arg_pair(arg: str) -> Tuple[str, str]:
    if ":=" not in arg:
        return arg, ""
    return arg.split(":=", 1)


def launch_args_to_dict(args: List[str]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for arg in args:
        key, value = launch_arg_pair(str(arg))
        if key:
            result[key] = value
    return result


def discover_launch_arg_defaults(launch_path: Path) -> Dict[str, str]:
    defaults: Dict[str, str] = {}
    if not launch_path.exists():
        return defaults
    try:
        root = ET.parse(str(launch_path)).getroot()
    except ET.ParseError:
        return defaults
    for node in root.findall(".//arg"):
        name = str(node.attrib.get("name", "")).strip()
        if not name or name in defaults:
            continue
        defaults[name] = str(node.attrib.get("default", ""))
    return defaults


def parse_host_defaults(config_path: Path) -> Dict[str, str]:
    defaults = {
        "agent_number": "4",
        "robot_ids": "1,2,3,4,5,6",
        "shape_type": "rectangle",
        "shape_scale": "1.0",
    }
    if not config_path.exists():
        return defaults

    text = config_path.read_text(encoding="utf-8", errors="ignore")
    patterns = {
        "agent_number": r'"agent_number:=([^"]+)"',
        "robot_ids": r'"robot_ids:=\[([^\]]*)\]"',
        "shape_type": r'"shape_type:=([^"]+)"',
        "shape_scale": r'"shape_scale:=([^"]+)"',
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if match:
            defaults[key] = match.group(1).strip()
    return defaults


def _launch_arg_key(arg: str) -> str:
    if ":=" not in arg:
        return arg
    return arg.split(":=", 1)[0]


def merge_launch_args(base_args: List[str], override_args: List[str]) -> List[str]:
    merged = list(base_args)
    for candidate in override_args:
        key = _launch_arg_key(candidate)
        replaced = False
        for idx, existing in enumerate(merged):
            if _launch_arg_key(existing) == key:
                merged[idx] = candidate
                replaced = True
                break
        if not replaced:
            merged.append(candidate)
    return merged


def load_host_config_snapshot(config_path: Path, ws_root: Path) -> Dict[str, object]:
    snapshot: Dict[str, object] = dict(SCRIPT_ARG_DEFAULTS)
    snapshot["host_launch_args"] = []
    if not config_path.exists():
        return snapshot

    cfg_prints = []
    for env_name in SCRIPT_ARG_VAR_MAP:
        cfg_prints.append(f'printf \'__CFG__ {env_name}=%s\\n\' "${{{env_name}-}}"')
    cfg_print_block = "\n".join(cfg_prints)
    command = f"""
set -e
source {shlex.quote(str(config_path))}
{cfg_print_block}
for arg in "${{HOST_LAUNCH_ARGS[@]}}"; do
  printf '__ARG__ %s\\n' "$arg"
done
"""
    proc = subprocess.run(
        ["bash", "-lc", command],
        cwd=str(ws_root),
        text=True,
        capture_output=True,
        check=True,
    )

    host_launch_args: List[str] = []
    for raw_line in proc.stdout.splitlines():
        line = raw_line.strip()
        if line.startswith("__CFG__ "):
            payload = line[len("__CFG__ ") :]
            if "=" in payload:
                key, value = payload.split("=", 1)
                if key in SCRIPT_ARG_VAR_MAP:
                    snapshot[SCRIPT_ARG_VAR_MAP[key]] = value
        elif line.startswith("__ARG__ "):
            host_launch_args.append(line[len("__ARG__ ") :])

    snapshot["host_launch_args"] = host_launch_args
    return snapshot


def _bash_single_quote(value: str) -> str:
    return "'" + str(value).replace("'", "'\"'\"'") + "'"


def write_host_config(config_path: Path, script_values: Dict[str, str], host_launch_args: List[str]) -> None:
    env_by_key = {key: env_name for key, env_name, _default in SCRIPT_ARG_DEFS}
    lines = [
        "# Host startup config for start_host.sh.",
        "# This file is sourced by bash. Keep it shell-compatible.",
        "",
    ]
    for key in SCRIPT_ARG_KEYS:
        env_name = env_by_key[key]
        value = str(script_values.get(key, SCRIPT_ARG_DEFAULTS.get(key, "")))
        lines.append(f"{env_name}={_bash_single_quote(value)}")
    lines.extend(
        [
            "",
            "# All entries are forwarded to turn_on_wheeltec_robot/shape_assembly_host.launch.",
            "HOST_LAUNCH_ARGS=(",
        ]
    )
    for arg in host_launch_args:
        lines.append(f"  {_bash_single_quote(arg)}")
    lines.extend([")", ""])
    config_path.write_text("\n".join(lines), encoding="utf-8")


class RosInterface:
    def __init__(self, app: "HostControlUI") -> None:
        self.app = app
        self.node_ready = False
        self.gather_pub = None
        self.current_ids: List[int] = []
        self.robot_odom_subs = {}
        self.robot_status_subs = {}
        self.robot_force_move_base_pubs = {}
        self.robot_move_base_cancel_pubs = {}
        self.global_subs = []

    def ensure_node(self) -> bool:
        if self.node_ready:
            return True
        if not rosgraph.is_master_online():
            return False

        rospy.init_node("host_control_ui", anonymous=True, disable_signals=True)
        self.gather_pub = rospy.Publisher("/gather_signal", UInt8, queue_size=1)
        self.global_subs = [
            rospy.Subscriber("/shape_assembly/task", ShapeTask, self._task_cb, queue_size=2),
            rospy.Subscriber("/gather_started", UInt8, self._gather_started_cb, queue_size=2),
            rospy.Subscriber("/gather_signal", UInt8, self._gather_signal_cb, queue_size=10),
        ]
        self.node_ready = True
        self.reconfigure_robot_subs(self.app.get_configured_robot_ids())
        return True

    def reconfigure_robot_subs(self, robot_ids: List[int]) -> None:
        if not self.node_ready:
            return
        target_ids = sorted(set(robot_ids))
        current_ids = set(self.current_ids)
        target_set = set(target_ids)

        for rid in sorted(current_ids - target_set):
            sub = self.robot_odom_subs.pop(rid, None)
            if sub is not None:
                sub.unregister()
            sub = self.robot_status_subs.pop(rid, None)
            if sub is not None:
                sub.unregister()
            self.robot_force_move_base_pubs.pop(rid, None)
            self.robot_move_base_cancel_pubs.pop(rid, None)

        for rid in sorted(target_set - current_ids):
            self.robot_odom_subs[rid] = rospy.Subscriber(
                f"/robot{rid}/odom",
                Odometry,
                self._odom_cb,
                callback_args=rid,
                queue_size=20,
            )
            self.robot_status_subs[rid] = rospy.Subscriber(
                f"/robot{rid}/shape_assembly/status",
                RobotFormationStatus,
                self._robot_status_cb,
                callback_args=rid,
                queue_size=20,
            )
            self.robot_force_move_base_pubs[rid] = rospy.Publisher(
                f"/robot{rid}/shape_assembly/force_move_base_mode",
                Bool,
                queue_size=1,
                latch=True,
            )
            self.robot_move_base_cancel_pubs[rid] = rospy.Publisher(
                f"/robot{rid}/move_base/cancel",
                GoalID,
                queue_size=1,
            )

        self.current_ids = target_ids

    def publish_gather_signal(self) -> bool:
        if not self.ensure_node():
            return False
        msg = UInt8()
        msg.data = 2
        self.gather_pub.publish(msg)
        return True

    def stop_gather_replanning(self) -> bool:
        if not self.ensure_node():
            return False
        rospy.set_param("/shape_assembly/stop_path_planning", True)
        rospy.set_param("/shape_assembly/active_robot_ids", sorted(set(self.current_ids)))
        return True

    def cancel_move_base_goals(self, robot_ids: List[int]) -> bool:
        if not self.ensure_node():
            return False
        self.reconfigure_robot_subs(robot_ids)
        cancel_msg = GoalID()
        cancel_msg.stamp = rospy.Time(0)
        cancel_msg.id = ""

        any_success = False
        errors = []
        for _ in range(3):
            for rid in sorted(set(robot_ids)):
                pub = self.robot_move_base_cancel_pubs.get(rid)
                if pub is None:
                    pub = rospy.Publisher(
                        f"/robot{rid}/move_base/cancel",
                        GoalID,
                        queue_size=1,
                    )
                    self.robot_move_base_cancel_pubs[rid] = pub
                try:
                    pub.publish(cancel_msg)
                    any_success = True
                except Exception as exc:
                    errors.append(f"robot{rid} move_base cancel failed: {exc}")
            time.sleep(0.05)

        if errors:
            self.app.on_force_move_base_feedback(errors)
        return any_success

    def apply_shape_params(self, shape_type: str, shape_scale: float) -> bool:
        if not self.ensure_node():
            return False
        rospy.set_param("/shape_task_supervisor/shape_type", str(shape_type).strip())
        rospy.set_param("/shape_task_supervisor/shape_scale", float(shape_scale))
        return True

    def set_force_move_base(self, robot_ids: List[int], enabled: bool) -> bool:
        if not self.ensure_node():
            return False
        self.reconfigure_robot_subs(robot_ids)
        any_success = False
        errors = []
        for rid in sorted(set(robot_ids)):
            pub = self.robot_force_move_base_pubs.get(rid)
            if pub is None:
                pub = rospy.Publisher(
                    f"/robot{rid}/shape_assembly/force_move_base_mode",
                    Bool,
                    queue_size=1,
                    latch=True,
                )
                self.robot_force_move_base_pubs[rid] = pub
            try:
                pub.publish(Bool(data=bool(enabled)))
                any_success = True
            except Exception as exc:
                errors.append(f"robot{rid} topic publish failed: {exc}")
            try:
                if DynamicReconfigureClient is None:
                    errors.append(f"robot{rid} dynreconf unavailable in host UI environment")
                else:
                    client = DynamicReconfigureClient(f"/robot{rid}/shape_assembly_swarm", timeout=1.0)
                    client.update_configuration({"force_move_base_mode": bool(enabled)})
                    any_success = True
            except Exception as exc:
                errors.append(f"robot{rid} dynreconf failed: {exc}")
        if errors:
            self.app.on_force_move_base_feedback(errors)
        return any_success

    def enter_force_move_base_mode(self, robot_ids: List[int]) -> bool:
        if not self.ensure_node():
            return False
        self.reconfigure_robot_subs(robot_ids)
        self.stop_gather_replanning()
        self.cancel_move_base_goals(robot_ids)
        success = self.set_force_move_base(robot_ids, True)
        self.stop_gather_replanning()
        return success

    def _task_cb(self, msg: ShapeTask) -> None:
        self.app.on_shape_task(msg)

    def _gather_started_cb(self, msg: UInt8) -> None:
        self.app.on_gather_started(int(msg.data))

    def _gather_signal_cb(self, msg: UInt8) -> None:
        self.app.on_gather_signal(int(msg.data))

    def _odom_cb(self, _msg: Odometry, robot_id: int) -> None:
        self.app.on_robot_heartbeat(robot_id)

    def _robot_status_cb(self, msg: RobotFormationStatus, robot_id: int) -> None:
        self.app.on_robot_status(robot_id, msg)


class HostControlUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.ws_root = Path(__file__).resolve().parents[3]
        self.start_host_script = self.ws_root / "start_host.sh"
        self.config_file = self.ws_root / "config" / "host_start.conf"
        self.config_snapshot = load_host_config_snapshot(self.config_file, self.ws_root)
        self.config_mtime: Optional[float] = self.config_file.stat().st_mtime if self.config_file.exists() else None
        self.host_launch_path = (
            self.ws_root
            / "src"
            / "turn_on_ws"
            / "src"
            / "turn_on_wheeltec_robot"
            / "launch"
            / "shape_assembly_host.launch"
        )
        self.launch_arg_defaults = discover_launch_arg_defaults(self.host_launch_path)
        self.host_arg_order: List[str] = []
        self.script_arg_vars: Dict[str, tk.StringVar] = {}
        self.host_arg_vars: Dict[str, tk.StringVar] = {}
        self.param_window: Optional[tk.Toplevel] = None

        self.host_process: Optional[subprocess.Popen] = None
        self.host_output_queue: "queue.Queue[str]" = queue.Queue()
        self.robot_states: Dict[int, Dict[str, object]] = {}
        self.last_task: Optional[ShapeTask] = None
        self.last_gather_started: Optional[int] = None
        self.last_gather_signal: Optional[int] = None
        self.monitored_ids: List[int] = []

        self.ros = RosInterface(self)

        self.root.title("Host Control UI")
        self.root.geometry("1180x760")

        host_args = launch_args_to_dict(list(self.config_snapshot.get("host_launch_args", [])))
        fallback_defaults = parse_host_defaults(self.config_file)
        self.agent_number_var = tk.StringVar(value=host_args.get("agent_number", fallback_defaults["agent_number"]))
        self.robot_ids_var = tk.StringVar(value=host_args.get("robot_ids", fallback_defaults["robot_ids"]))
        self.shape_type_var = tk.StringVar(value=host_args.get("shape_type", fallback_defaults["shape_type"]))
        self.shape_scale_var = tk.StringVar(value=host_args.get("shape_scale", fallback_defaults["shape_scale"]))
        self.master_status_var = tk.StringVar(value="ROS master: offline")
        self.host_status_var = tk.StringVar(value="Host process: idle")
        self.gather_status_var = tk.StringVar(value="Gather: unknown")
        self.task_status_var = tk.StringVar(value="Task: none")
        self.config_status_var = tk.StringVar(value="Config: loaded")
        self.force_move_base_status_var = tk.StringVar(value="Force MoveBase: unknown")
        self.last_force_move_base: Optional[bool] = None
        self._init_param_vars_from_snapshot(self.config_snapshot)

        self._build_ui()
        self._sync_agent_number_from_ids()
        self._schedule_tick()

    def _init_param_vars_from_snapshot(self, snapshot: Dict[str, object]) -> None:
        for key in SCRIPT_ARG_KEYS:
            value = str(snapshot.get(key, SCRIPT_ARG_DEFAULTS.get(key, "")) or "")
            self.script_arg_vars[key] = tk.StringVar(value=value)

        arg_values = dict(self.launch_arg_defaults)
        arg_values.update(launch_args_to_dict(list(snapshot.get("host_launch_args", []))))
        arg_values["agent_number"] = self.agent_number_var.get()
        arg_values["robot_ids"] = self.robot_ids_var.get()
        arg_values["shape_type"] = self.shape_type_var.get()
        arg_values["shape_scale"] = self.shape_scale_var.get()

        ordered = list(self.launch_arg_defaults.keys())
        for key in list(arg_values.keys()):
            if key not in ordered:
                ordered.append(key)
        self.host_arg_order = ordered

        self.host_arg_vars.clear()
        for key in ordered:
            if key == "agent_number":
                self.host_arg_vars[key] = self.agent_number_var
            elif key == "robot_ids":
                self.host_arg_vars[key] = self.robot_ids_var
            elif key == "shape_type":
                self.host_arg_vars[key] = self.shape_type_var
            elif key == "shape_scale":
                self.host_arg_vars[key] = self.shape_scale_var
            else:
                self.host_arg_vars[key] = tk.StringVar(value=str(arg_values.get(key, "")))

    def _sync_common_host_args_from_main_fields(self) -> None:
        if "agent_number" in self.host_arg_vars:
            self.host_arg_vars["agent_number"].set(self.agent_number_var.get().strip())
        if "robot_ids" in self.host_arg_vars:
            self.host_arg_vars["robot_ids"].set(self.robot_ids_var.get().strip())
        if "shape_type" in self.host_arg_vars:
            self.host_arg_vars["shape_type"].set(self.shape_type_var.get().strip())
        if "shape_scale" in self.host_arg_vars:
            self.host_arg_vars["shape_scale"].set(self.shape_scale_var.get().strip())

    def _script_args_from_vars(self) -> Dict[str, str]:
        return {key: var.get().strip() for key, var in self.script_arg_vars.items()}

    def _host_launch_args_from_vars(self, include_empty: bool = False) -> List[str]:
        self._sync_common_host_args_from_main_fields()
        args = []
        for key in self.host_arg_order:
            var = self.host_arg_vars.get(key)
            if var is None:
                continue
            value = var.get().strip()
            if not include_empty and value == "":
                continue
            args.append(f"{key}:={value}")
        return args

    def _profile_data(self) -> Dict[str, object]:
        return {
            "script_args": self._script_args_from_vars(),
            "host_launch_args": launch_args_to_dict(self._host_launch_args_from_vars(include_empty=True)),
        }

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)
        self.root.rowconfigure(3, weight=1)

        config_frame = ttk.LabelFrame(self.root, text="Experiment Config")
        config_frame.grid(row=0, column=0, sticky="nsew", padx=12, pady=(12, 6))
        for idx in range(8):
            config_frame.columnconfigure(idx, weight=1 if idx in (1, 3, 5, 7) else 0)

        ttk.Label(config_frame, text="Robot IDs").grid(row=0, column=0, sticky="w", padx=8, pady=8)
        robot_ids_entry = ttk.Entry(config_frame, textvariable=self.robot_ids_var)
        robot_ids_entry.grid(row=0, column=1, sticky="ew", padx=8, pady=8)
        robot_ids_entry.bind("<KeyRelease>", lambda _event: self._sync_agent_number_from_ids())

        ttk.Label(config_frame, text="Robot Count").grid(row=0, column=2, sticky="w", padx=8, pady=8)
        ttk.Entry(config_frame, textvariable=self.agent_number_var).grid(row=0, column=3, sticky="ew", padx=8, pady=8)

        ttk.Label(config_frame, text="Shape").grid(row=0, column=4, sticky="w", padx=8, pady=8)
        shape_box = ttk.Combobox(config_frame, textvariable=self.shape_type_var, values=SHAPE_CHOICES)
        shape_box.grid(row=0, column=5, sticky="ew", padx=8, pady=8)

        ttk.Label(config_frame, text="Scale").grid(row=0, column=6, sticky="w", padx=8, pady=8)
        ttk.Entry(config_frame, textvariable=self.shape_scale_var).grid(row=0, column=7, sticky="ew", padx=8, pady=8)

        button_frame = ttk.Frame(config_frame)
        button_frame.grid(row=1, column=0, columnspan=8, sticky="ew", padx=8, pady=(0, 8))
        for idx in range(8):
            button_frame.columnconfigure(idx, weight=1)

        ttk.Button(button_frame, text="Start Host", command=self._start_host).grid(row=0, column=0, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Stop Host", command=self._stop_host).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Apply Shape", command=self._apply_shape).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Send Gather=2", command=self._send_gather_signal).grid(row=0, column=3, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Refresh Monitors", command=self._refresh_monitor_ids).grid(row=0, column=4, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Force MoveBase ON", command=self._force_move_base_on).grid(row=0, column=5, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Force MoveBase OFF", command=self._force_move_base_off).grid(row=0, column=6, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Parameters", command=self._open_param_editor).grid(row=0, column=7, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Hot Reload Config", command=self._hot_reload_config).grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(button_frame, text="Load Profile", command=self._load_profile).grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(button_frame, text="Save Profile", command=self._save_profile).grid(row=1, column=4, columnspan=2, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(button_frame, text="Save host_start.conf", command=self._save_host_start_config).grid(row=1, column=6, columnspan=2, sticky="ew", padx=4, pady=(6, 0))

        summary_frame = ttk.LabelFrame(self.root, text="Host Summary")
        summary_frame.grid(row=1, column=0, sticky="nsew", padx=12, pady=6)
        summary_frame.columnconfigure(1, weight=1)

        ttk.Label(summary_frame, textvariable=self.master_status_var).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.host_status_var).grid(row=0, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.gather_status_var).grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.task_status_var).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.config_status_var).grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.force_move_base_status_var).grid(row=2, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(
            summary_frame,
            text="start_host.sh -> shape_assembly_real_robot.sh -> shape_assembly_host.launch -> fm2_gather + shape_task_supervisor",
        ).grid(row=3, column=0, columnspan=2, sticky="w", padx=8, pady=(4, 8))

        table_frame = ttk.LabelFrame(self.root, text="Robot Status")
        table_frame.grid(row=2, column=0, sticky="nsew", padx=12, pady=6)
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        columns = ("robot", "connection", "heartbeat", "phase", "task_id", "shape_active", "note")
        self.robot_table = ttk.Treeview(table_frame, columns=columns, show="headings", height=10)
        headings = {
            "robot": "Robot",
            "connection": "Connection",
            "heartbeat": "Last Msg",
            "phase": "Task Phase",
            "task_id": "Task ID",
            "shape_active": "Shape Active",
            "note": "Note",
        }
        widths = {"robot": 70, "connection": 110, "heartbeat": 90, "phase": 140, "task_id": 80, "shape_active": 100, "note": 200}
        for column in columns:
            self.robot_table.heading(column, text=headings[column])
            self.robot_table.column(column, width=widths[column], anchor="center")
        self.robot_table.grid(row=0, column=0, sticky="nsew")
        table_scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.robot_table.yview)
        table_scrollbar.grid(row=0, column=1, sticky="ns")
        self.robot_table.configure(yscrollcommand=table_scrollbar.set)

        log_frame = ttk.LabelFrame(self.root, text="Host Log")
        log_frame.grid(row=3, column=0, sticky="nsew", padx=12, pady=(6, 12))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap="word", height=12, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")

    def _make_scrollable_tab(self, notebook: ttk.Notebook, title: str) -> ttk.Frame:
        outer = ttk.Frame(notebook)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)
        canvas = tk.Canvas(outer, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        window_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window_id, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        notebook.add(outer, text=title)
        return inner

    def _populate_param_rows(self, parent: ttk.Frame, keys: List[str], vars_by_key: Dict[str, tk.StringVar]) -> None:
        parent.columnconfigure(1, weight=1)
        parent.columnconfigure(2, weight=1)
        ttk.Label(parent, text="Parameter").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Label(parent, text="Value").grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(parent, text="Default").grid(row=0, column=2, sticky="ew", padx=8, pady=(8, 4))
        for row, key in enumerate(keys, start=1):
            var = vars_by_key.get(key)
            if var is None:
                continue
            ttk.Label(parent, text=key).grid(row=row, column=0, sticky="w", padx=8, pady=3)
            ttk.Entry(parent, textvariable=var).grid(row=row, column=1, sticky="ew", padx=8, pady=3)
            default = SCRIPT_ARG_DEFAULTS.get(key, self.launch_arg_defaults.get(key, ""))
            ttk.Label(parent, text=default).grid(row=row, column=2, sticky="ew", padx=8, pady=3)

    def _open_param_editor(self) -> None:
        if self.param_window is not None and self.param_window.winfo_exists():
            self.param_window.lift()
            self.param_window.focus_force()
            return

        window = tk.Toplevel(self.root)
        window.title("Host Parameters")
        window.geometry("980x720")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        self.param_window = window
        window.protocol("WM_DELETE_WINDOW", lambda: (setattr(self, "param_window", None), window.destroy()))

        notebook = ttk.Notebook(window)
        notebook.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        script_tab = self._make_scrollable_tab(notebook, "Start Script")
        self._populate_param_rows(script_tab, SCRIPT_ARG_KEYS, self.script_arg_vars)

        launch_tab = self._make_scrollable_tab(notebook, "Gather Launch")
        self._populate_param_rows(launch_tab, self.host_arg_order, self.host_arg_vars)

        action_frame = ttk.Frame(window)
        action_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))
        for idx in range(5):
            action_frame.columnconfigure(idx, weight=1)
        ttk.Button(action_frame, text="Load Profile", command=self._load_profile).grid(row=0, column=0, sticky="ew", padx=4)
        ttk.Button(action_frame, text="Save Profile", command=self._save_profile).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(action_frame, text="Save host_start.conf", command=self._save_host_start_config).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(action_frame, text="Reload host_start.conf", command=self._reload_config_snapshot).grid(row=0, column=3, sticky="ew", padx=4)
        ttk.Button(action_frame, text="Close", command=window.destroy).grid(row=0, column=4, sticky="ew", padx=4)

    def _schedule_tick(self) -> None:
        self._tick()
        self.root.after(300, self._schedule_tick)

    def _tick(self) -> None:
        master_online = rosgraph.is_master_online()
        self.master_status_var.set("ROS master: online" if master_online else "ROS master: offline")
        self._update_config_status()

        if master_online and self.ros.ensure_node():
            desired_ids = self.get_configured_robot_ids()
            if desired_ids != self.monitored_ids:
                self.ros.reconfigure_robot_subs(desired_ids)
                self.monitored_ids = desired_ids

        self._drain_host_output()
        self._update_host_process_status()
        self._refresh_robot_table()
        self._refresh_task_summary()

    def _sync_agent_number_from_ids(self) -> None:
        try:
            self.agent_number_var.set(str(len(parse_robot_ids(self.robot_ids_var.get()))))
        except Exception:
            pass

    def _refresh_monitor_ids(self) -> None:
        desired_ids = self.get_configured_robot_ids()
        self.monitored_ids = desired_ids
        if self.ros.node_ready:
            self.ros.reconfigure_robot_subs(desired_ids)

    def _update_config_status(self) -> None:
        if not self.config_file.exists():
            self.config_status_var.set("Config: missing config/host_start.conf")
            return
        current_mtime = self.config_file.stat().st_mtime
        if self.config_mtime is None:
            self.config_mtime = current_mtime
        if current_mtime > (self.config_mtime + 1e-6):
            self.config_status_var.set("Config: file changed on disk, click Hot Reload Config")
        else:
            self.config_status_var.set("Config: synced with host_start.conf")

    def get_configured_robot_ids(self) -> List[int]:
        try:
            return parse_robot_ids(self.robot_ids_var.get())
        except Exception:
            return []

    def _start_host(self) -> None:
        if self.host_process is not None and self.host_process.poll() is None:
            messagebox.showinfo("Host Running", "start_host.sh is already running.")
            return

        try:
            robot_ids = parse_robot_ids(self.robot_ids_var.get())
            agent_number = int(self.agent_number_var.get())
            shape_scale = float(self.shape_scale_var.get())
        except Exception as exc:
            messagebox.showerror("Invalid Config", str(exc))
            return

        if agent_number != len(robot_ids):
            agent_number = len(robot_ids)
            self.agent_number_var.set(str(agent_number))
        self._sync_common_host_args_from_main_fields()

        cmd = self._build_start_host_command()
        env = os.environ.copy()
        self.host_process = subprocess.Popen(
            cmd,
            cwd=str(self.ws_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
            env=env,
        )
        self._append_log("[ui] Started: {}\n".format(" ".join(cmd)))
        threading.Thread(target=self._read_host_output, daemon=True).start()

    def _build_start_host_command(self) -> List[str]:
        script_args = []
        for key in SCRIPT_ARG_KEYS:
            var = self.script_arg_vars.get(key)
            value = var.get().strip() if var is not None else ""
            if value:
                script_args.append(f"{key}:={value}")

        return [str(self.start_host_script)] + script_args + self._host_launch_args_from_vars()

    def _stop_host(self) -> None:
        if self.host_process is None or self.host_process.poll() is not None:
            return
        try:
            os.killpg(self.host_process.pid, signal.SIGTERM)
            self._append_log("[ui] Sent SIGTERM to start_host.sh process group\n")
        except ProcessLookupError:
            pass

    def _apply_shape(self) -> None:
        try:
            shape_scale = float(self.shape_scale_var.get())
        except ValueError as exc:
            messagebox.showerror("Invalid Shape Scale", str(exc))
            return
        if not self.ros.apply_shape_params(self.shape_type_var.get(), shape_scale):
            messagebox.showwarning("ROS Offline", "ROS master or shape_task_supervisor is not ready.")
            return
        self._append_log(
            "[ui] Applied shape params: type={} scale={:.3f}\n".format(
                self.shape_type_var.get().strip(),
                shape_scale,
            )
        )

    def _send_gather_signal(self) -> None:
        try:
            shape_scale = float(self.shape_scale_var.get())
        except ValueError as exc:
            messagebox.showerror("Invalid Shape Scale", str(exc))
            return
        if not self.ros.apply_shape_params(self.shape_type_var.get(), shape_scale):
            messagebox.showwarning("ROS Offline", "ROS master or shape_task_supervisor is not ready.")
            return
        if not self.ros.publish_gather_signal():
            messagebox.showwarning("ROS Offline", "ROS master is not ready. Cannot publish /gather_signal.")
            return
        self._append_log(
            "[ui] Applied shape params type={} scale={:.3f}, then published /gather_signal = 2\n".format(
                self.shape_type_var.get().strip(),
                shape_scale,
            )
        )

    def _set_force_move_base(self, enabled: bool) -> None:
        robot_ids = self.get_configured_robot_ids()
        if not robot_ids:
            messagebox.showwarning("No Robots", "Configured robot_ids is empty or invalid.")
            return
        if enabled:
            success = self.ros.enter_force_move_base_mode(robot_ids)
        else:
            success = self.ros.set_force_move_base(robot_ids, enabled)

        if not success:
            messagebox.showwarning(
                "ROS Offline",
                "ROS master is not ready or shape_assembly_swarm dynamic_reconfigure is unavailable.",
            )
            return
        self.last_force_move_base = bool(enabled)
        state = "ON" if enabled else "OFF"
        self.force_move_base_status_var.set(f"Force MoveBase: {state}")
        if enabled:
            self._append_log(
                "[ui] Stopped gather replanning, cancelled move_base goals, then set force_move_base_mode=true for robots {}\n".format(
                    robot_ids
                )
            )
        else:
            self._append_log("[ui] Set force_move_base_mode=false for robots {}\n".format(robot_ids))

    def _force_move_base_on(self) -> None:
        self._set_force_move_base(True)

    def _force_move_base_off(self) -> None:
        self._set_force_move_base(False)

    def _apply_profile_data(self, data: Dict[str, object]) -> None:
        script_args = data.get("script_args", {})
        if isinstance(script_args, dict):
            for key, value in script_args.items():
                if key not in self.script_arg_vars:
                    self.script_arg_vars[key] = tk.StringVar(value="")
                self.script_arg_vars[key].set(str(value))

        host_args = data.get("host_launch_args", {})
        if isinstance(host_args, list):
            host_args = launch_args_to_dict([str(item) for item in host_args])
        if isinstance(host_args, dict):
            for key, value in host_args.items():
                if key not in self.host_arg_order:
                    self.host_arg_order.append(str(key))
                if key not in self.host_arg_vars:
                    self.host_arg_vars[str(key)] = tk.StringVar(value="")
                self.host_arg_vars[str(key)].set(str(value))

        self._sync_agent_number_from_ids()
        self._sync_common_host_args_from_main_fields()
        self._refresh_monitor_ids()

    def _load_profile(self) -> None:
        path = filedialog.askopenfilename(
            title="Load host parameter profile",
            initialdir=str(self.ws_root / "config"),
            filetypes=[("JSON profiles", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("profile root must be a JSON object")
            self._apply_profile_data(data)
        except Exception as exc:
            messagebox.showerror("Load Profile Failed", str(exc))
            return
        self._append_log("[ui] Loaded parameter profile: {}\n".format(path))

    def _save_profile(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save host parameter profile",
            initialdir=str(self.ws_root / "config"),
            defaultextension=".json",
            filetypes=[("JSON profiles", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            Path(path).write_text(json.dumps(self._profile_data(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Save Profile Failed", str(exc))
            return
        self._append_log("[ui] Saved parameter profile: {}\n".format(path))

    def _save_host_start_config(self) -> None:
        try:
            write_host_config(
                self.config_file,
                self._script_args_from_vars(),
                self._host_launch_args_from_vars(include_empty=False),
            )
            self.config_snapshot = load_host_config_snapshot(self.config_file, self.ws_root)
            self.config_mtime = self.config_file.stat().st_mtime if self.config_file.exists() else None
        except Exception as exc:
            messagebox.showerror("Save Config Failed", str(exc))
            return
        self._append_log("[ui] Saved active parameters to {}\n".format(self.config_file))

    def _reload_config_snapshot(self) -> None:
        self.config_snapshot = load_host_config_snapshot(self.config_file, self.ws_root)
        self.config_mtime = self.config_file.stat().st_mtime if self.config_file.exists() else None
        host_args = launch_args_to_dict(list(self.config_snapshot.get("host_launch_args", [])))
        fallback_defaults = parse_host_defaults(self.config_file)
        self.agent_number_var.set(host_args.get("agent_number", fallback_defaults["agent_number"]))
        self.robot_ids_var.set(host_args.get("robot_ids", fallback_defaults["robot_ids"]))
        self.shape_type_var.set(host_args.get("shape_type", fallback_defaults["shape_type"]))
        self.shape_scale_var.set(host_args.get("shape_scale", fallback_defaults["shape_scale"]))
        self._init_param_vars_from_snapshot(self.config_snapshot)
        if self.param_window is not None and self.param_window.winfo_exists():
            self.param_window.destroy()
            self.param_window = None
        self._sync_agent_number_from_ids()
        self._refresh_monitor_ids()
        self._append_log("[ui] Reloaded config from {}\n".format(self.config_file))

    def _hot_reload_config(self) -> None:
        was_running = self.host_process is not None and self.host_process.poll() is None
        self._reload_config_snapshot()
        if not was_running:
            return

        self._append_log("[ui] Host is running, restart to apply startup-only params (for example robot_ids)\n")
        self._stop_host()
        deadline = time.time() + 8.0
        while self.host_process is not None and self.host_process.poll() is None and time.time() < deadline:
            self.root.update()
            time.sleep(0.05)
        self._start_host()

    def _read_host_output(self) -> None:
        if self.host_process is None or self.host_process.stdout is None:
            return
        for line in self.host_process.stdout:
            self.host_output_queue.put(line)

    def _drain_host_output(self) -> None:
        while True:
            try:
                line = self.host_output_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _update_host_process_status(self) -> None:
        if self.host_process is None:
            self.host_status_var.set("Host process: idle")
            return
        code = self.host_process.poll()
        if code is None:
            self.host_status_var.set(f"Host process: running (pid={self.host_process.pid})")
        else:
            self.host_status_var.set(f"Host process: exited ({code})")

    def _refresh_robot_table(self) -> None:
        desired_ids = self.get_configured_robot_ids()
        now = time.time()
        existing = set(self.robot_table.get_children())
        wanted = {str(rid) for rid in desired_ids}
        for item in sorted(existing - wanted):
            self.robot_table.delete(item)

        for rid in desired_ids:
            item_id = str(rid)
            state = self.robot_states.setdefault(rid, {})
            last_heartbeat = float(state.get("last_heartbeat", 0.0) or 0.0)
            age = now - last_heartbeat if last_heartbeat > 0.0 else None
            if age is None:
                connection = "No Msg"
                age_text = "-"
            elif age <= CONNECTION_TIMEOUT_SEC:
                connection = "Online"
                age_text = "{:.1f}s".format(age)
            elif age <= STALE_TIMEOUT_SEC:
                connection = "Stale"
                age_text = "{:.1f}s".format(age)
            else:
                connection = "Offline"
                age_text = "{:.1f}s".format(age)

            phase = state.get("phase_label", "-")
            task_id = str(state.get("task_id", "-"))
            shape_active = "Yes" if state.get("shape_active", False) else "No"
            note = str(state.get("note", "-"))
            values = (rid, connection, age_text, phase, task_id, shape_active, note)
            if item_id in existing:
                self.robot_table.item(item_id, values=values)
            else:
                self.robot_table.insert("", "end", iid=item_id, values=values)

    def _refresh_task_summary(self) -> None:
        if self.last_force_move_base is None:
            self.force_move_base_status_var.set("Force MoveBase: unknown")
        elif self.last_force_move_base:
            self.force_move_base_status_var.set("Force MoveBase: ON")
        else:
            self.force_move_base_status_var.set("Force MoveBase: OFF")
        if self.last_gather_started == 1:
            self.gather_status_var.set("Gather: started")
        elif self.last_gather_started == 0:
            self.gather_status_var.set("Gather: idle")
        elif self.last_gather_signal == 2:
            self.gather_status_var.set("Gather: compute requested")
        else:
            self.gather_status_var.set("Gather: unknown")

        if self.last_task is None:
            self.task_status_var.set("Task: none")
            return

        center = self.last_task.center.position
        heading_deg = self.last_task.shape_heading * 180.0 / 3.141592653589793
        self.task_status_var.set(
            "Task: id={} shape={} scale={:.2f} center=({:.2f}, {:.2f}) heading={:.1f}deg replan={}".format(
                self.last_task.task_id,
                self.last_task.shape_type,
                self.last_task.shape_scale,
                center.x,
                center.y,
                heading_deg,
                str(bool(self.last_task.replan)).lower(),
            )
        )

    def on_shape_task(self, msg: ShapeTask) -> None:
        self.last_task = msg

    def on_gather_started(self, value: int) -> None:
        self.last_gather_started = value

    def on_gather_signal(self, value: int) -> None:
        self.last_gather_signal = value

    def on_robot_heartbeat(self, robot_id: int) -> None:
        state = self.robot_states.setdefault(robot_id, {})
        state["last_heartbeat"] = time.time()

    def on_robot_status(self, robot_id: int, msg: RobotFormationStatus) -> None:
        state = self.robot_states.setdefault(robot_id, {})
        state["last_heartbeat"] = time.time()
        state["task_id"] = int(msg.task_id)
        state["phase"] = int(msg.phase)
        state["phase_label"] = PHASE_LABELS.get(int(msg.phase), f"Phase{int(msg.phase)}")
        state["shape_active"] = bool(msg.shape_active)
        state["note"] = msg.note or "-"

    def on_force_move_base_feedback(self, errors: List[str]) -> None:
        if not errors:
            return
        self._append_log("[ui] force_move_base warnings:\n")
        for item in errors:
            self._append_log(f"  - {item}\n")

    def on_close(self) -> None:
        self._stop_host()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    app = HostControlUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
