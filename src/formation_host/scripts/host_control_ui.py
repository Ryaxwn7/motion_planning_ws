#!/usr/bin/env python3

import os
import queue
import re
import signal
import subprocess
import threading
import time
import shlex
from pathlib import Path
from typing import Dict, List, Optional

import rosgraph
import rospy
import tkinter as tk
from formation_msgs.msg import RobotFormationStatus, ShapeTask
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from tkinter import messagebox, scrolledtext, ttk


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
    snapshot: Dict[str, object] = {
        "start_roscore": "false",
        "launch_map_server": "true",
        "auto_start_gather": "false",
        "roscore_wait": "8.0",
        "ros_master_uri": "",
        "ros_ip": "",
        "map_file": "",
        "host_launch_args": [],
    }
    if not config_path.exists():
        return snapshot

    command = f"""
set -e
source {shlex.quote(str(config_path))}
printf '__CFG__ START_ROSCORE=%s\\n' "${{START_ROSCORE-}}"
printf '__CFG__ LAUNCH_MAP_SERVER=%s\\n' "${{LAUNCH_MAP_SERVER-}}"
printf '__CFG__ AUTO_START_GATHER=%s\\n' "${{AUTO_START_GATHER-}}"
printf '__CFG__ ROSCORE_WAIT=%s\\n' "${{ROSCORE_WAIT-}}"
printf '__CFG__ ROS_MASTER_URI_VALUE=%s\\n' "${{ROS_MASTER_URI_VALUE-}}"
printf '__CFG__ ROS_IP_VALUE=%s\\n' "${{ROS_IP_VALUE-}}"
printf '__CFG__ MAP_FILE_VALUE=%s\\n' "${{MAP_FILE_VALUE-}}"
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
    mapping = {
        "START_ROSCORE": "start_roscore",
        "LAUNCH_MAP_SERVER": "launch_map_server",
        "AUTO_START_GATHER": "auto_start_gather",
        "ROSCORE_WAIT": "roscore_wait",
        "ROS_MASTER_URI_VALUE": "ros_master_uri",
        "ROS_IP_VALUE": "ros_ip",
        "MAP_FILE_VALUE": "map_file",
    }
    for raw_line in proc.stdout.splitlines():
        line = raw_line.strip()
        if line.startswith("__CFG__ "):
            payload = line[len("__CFG__ ") :]
            if "=" in payload:
                key, value = payload.split("=", 1)
                if key in mapping:
                    snapshot[mapping[key]] = value
        elif line.startswith("__ARG__ "):
            host_launch_args.append(line[len("__ARG__ ") :])

    snapshot["host_launch_args"] = host_launch_args
    return snapshot


class RosInterface:
    def __init__(self, app: "HostControlUI") -> None:
        self.app = app
        self.node_ready = False
        self.gather_pub = None
        self.current_ids: List[int] = []
        self.robot_odom_subs = {}
        self.robot_status_subs = {}
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

        self.current_ids = target_ids

    def publish_gather_signal(self) -> bool:
        if not self.ensure_node():
            return False
        msg = UInt8()
        msg.data = 2
        self.gather_pub.publish(msg)
        return True

    def apply_shape_params(self, shape_type: str, shape_scale: float) -> bool:
        if not self.ensure_node():
            return False
        rospy.set_param("/shape_task_supervisor/shape_type", str(shape_type).strip())
        rospy.set_param("/shape_task_supervisor/shape_scale", float(shape_scale))
        return True

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
        defaults = parse_host_defaults(self.config_file)
        self.config_snapshot = load_host_config_snapshot(self.config_file, self.ws_root)
        self.config_mtime: Optional[float] = self.config_file.stat().st_mtime if self.config_file.exists() else None

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

        self.agent_number_var = tk.StringVar(value=defaults["agent_number"])
        self.robot_ids_var = tk.StringVar(value=defaults["robot_ids"])
        self.shape_type_var = tk.StringVar(value=defaults["shape_type"])
        self.shape_scale_var = tk.StringVar(value=defaults["shape_scale"])
        self.master_status_var = tk.StringVar(value="ROS master: offline")
        self.host_status_var = tk.StringVar(value="Host process: idle")
        self.gather_status_var = tk.StringVar(value="Gather: unknown")
        self.task_status_var = tk.StringVar(value="Task: none")
        self.config_status_var = tk.StringVar(value="Config: loaded")

        self._build_ui()
        self._sync_agent_number_from_ids()
        self._schedule_tick()

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
        for idx in range(5):
            button_frame.columnconfigure(idx, weight=1)

        ttk.Button(button_frame, text="Start Host", command=self._start_host).grid(row=0, column=0, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Stop Host", command=self._stop_host).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Apply Shape", command=self._apply_shape).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Send Gather=2", command=self._send_gather_signal).grid(row=0, column=3, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Refresh Monitors", command=self._refresh_monitor_ids).grid(row=0, column=4, sticky="ew", padx=4)
        ttk.Button(button_frame, text="Hot Reload Config", command=self._hot_reload_config).grid(row=1, column=0, columnspan=5, sticky="ew", padx=4, pady=(6, 0))

        summary_frame = ttk.LabelFrame(self.root, text="Host Summary")
        summary_frame.grid(row=1, column=0, sticky="nsew", padx=12, pady=6)
        summary_frame.columnconfigure(1, weight=1)

        ttk.Label(summary_frame, textvariable=self.master_status_var).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.host_status_var).grid(row=0, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.gather_status_var).grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.task_status_var).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(summary_frame, textvariable=self.config_status_var).grid(row=2, column=0, sticky="w", padx=8, pady=4)
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

        cmd = self._build_start_host_command(robot_ids, agent_number, shape_scale)
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

    def _build_start_host_command(self, robot_ids: List[int], agent_number: int, shape_scale: float) -> List[str]:
        base_args = list(self.config_snapshot.get("host_launch_args", []))
        override_args = [
            f"agent_number:={agent_number}",
            robot_ids_arg(robot_ids),
            f"shape_type:={self.shape_type_var.get().strip()}",
            f"shape_scale:={shape_scale}",
        ]
        merged_host_args = merge_launch_args(base_args, override_args)

        script_args = []
        mapping = [
            ("start_roscore", "start_roscore"),
            ("launch_map_server", "launch_map_server"),
            ("auto_start_gather", "auto_start_gather"),
            ("roscore_wait", "roscore_wait"),
            ("ros_master_uri", "ros_master_uri"),
            ("ros_ip", "ros_ip"),
            ("map_file", "map_file"),
        ]
        for snapshot_key, script_key in mapping:
            value = str(self.config_snapshot.get(snapshot_key, "") or "").strip()
            if value:
                script_args.append(f"{script_key}:={value}")

        return [str(self.start_host_script)] + script_args + merged_host_args

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
        if not self.ros.publish_gather_signal():
            messagebox.showwarning("ROS Offline", "ROS master is not ready. Cannot publish /gather_signal.")
            return
        self._append_log("[ui] Published /gather_signal = 2\n")

    def _reload_config_snapshot(self) -> None:
        self.config_snapshot = load_host_config_snapshot(self.config_file, self.ws_root)
        self.config_mtime = self.config_file.stat().st_mtime if self.config_file.exists() else None
        defaults = parse_host_defaults(self.config_file)
        self.agent_number_var.set(defaults["agent_number"])
        self.robot_ids_var.set(defaults["robot_ids"])
        self.shape_type_var.set(defaults["shape_type"])
        self.shape_scale_var.set(defaults["shape_scale"])
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
