#!/usr/bin/env python3

import json
import math
from pathlib import Path
from typing import Dict

import rospy
import tf.transformations
import tf2_ros
import tkinter as tk
from geometry_msgs.msg import TransformStamped
from tkinter import filedialog, messagebox, ttk


def _float_param(name: str, default: float) -> float:
    try:
        return float(rospy.get_param("~" + name, default))
    except Exception:
        return default


def _str_param(name: str, default: str) -> str:
    return str(rospy.get_param("~" + name, default)).strip() or default


class TfAdjustUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("TF Adjust: map -> world")
        self.root.geometry("620x360")

        self.parent_var = tk.StringVar(value=_str_param("parent_frame", "map"))
        self.child_var = tk.StringVar(value=_str_param("child_frame", "world"))
        self.rate_var = tk.StringVar(value=str(_float_param("rate", 30.0)))
        self.step_var = tk.StringVar(value="0.01")
        self.angle_step_deg_var = tk.StringVar(value="1.0")
        self.status_var = tk.StringVar(value="Publishing")

        self.vars: Dict[str, tk.StringVar] = {
            "x": tk.StringVar(value=str(_float_param("x", 0.0))),
            "y": tk.StringVar(value=str(_float_param("y", 0.0))),
            "z": tk.StringVar(value=str(_float_param("z", 0.0))),
            "roll": tk.StringVar(value=str(_float_param("roll", 0.0))),
            "pitch": tk.StringVar(value=str(_float_param("pitch", 0.0))),
            "yaw": tk.StringVar(value=str(_float_param("yaw", 0.0))),
        }
        self.deg_vars: Dict[str, tk.StringVar] = {
            key: tk.StringVar(value=self._rad_to_deg_text(self.vars[key].get()))
            for key in ("roll", "pitch", "yaw")
        }

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self._shutdown = False
        self._build_ui()
        self._schedule_publish()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)

        frame_box = ttk.LabelFrame(self.root, text="Frames")
        frame_box.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        for idx in range(6):
            frame_box.columnconfigure(idx, weight=1)
        ttk.Label(frame_box, text="Parent").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(frame_box, textvariable=self.parent_var).grid(row=0, column=1, sticky="ew", padx=6, pady=6)
        ttk.Label(frame_box, text="Child").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(frame_box, textvariable=self.child_var).grid(row=0, column=3, sticky="ew", padx=6, pady=6)
        ttk.Label(frame_box, text="Rate").grid(row=0, column=4, sticky="w", padx=6, pady=6)
        ttk.Entry(frame_box, textvariable=self.rate_var, width=8).grid(row=0, column=5, sticky="ew", padx=6, pady=6)

        value_box = ttk.LabelFrame(self.root, text="Transform")
        value_box.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        for idx in range(7):
            value_box.columnconfigure(idx, weight=1 if idx in (1, 4) else 0)
        ttk.Label(value_box, text="Axis").grid(row=0, column=0, padx=6, pady=4)
        ttk.Label(value_box, text="Value").grid(row=0, column=1, padx=6, pady=4)
        ttk.Label(value_box, text="").grid(row=0, column=2, padx=6, pady=4)
        ttk.Label(value_box, text="Degrees").grid(row=0, column=4, padx=6, pady=4)

        for row, key in enumerate(("x", "y", "z", "roll", "pitch", "yaw"), start=1):
            ttk.Label(value_box, text=key).grid(row=row, column=0, sticky="w", padx=6, pady=4)
            ttk.Entry(value_box, textvariable=self.vars[key], width=14).grid(row=row, column=1, sticky="ew", padx=6, pady=4)
            ttk.Button(value_box, text="-", width=3, command=lambda k=key: self._nudge(k, -1.0)).grid(
                row=row, column=2, padx=2, pady=4
            )
            ttk.Button(value_box, text="+", width=3, command=lambda k=key: self._nudge(k, 1.0)).grid(
                row=row, column=3, padx=2, pady=4
            )
            if key in self.deg_vars:
                ttk.Entry(value_box, textvariable=self.deg_vars[key], width=12).grid(
                    row=row, column=4, sticky="ew", padx=6, pady=4
                )
                ttk.Button(value_box, text="deg->rad", command=lambda k=key: self._apply_degrees(k)).grid(
                    row=row, column=5, columnspan=2, sticky="ew", padx=6, pady=4
                )

        control_box = ttk.LabelFrame(self.root, text="Step / Files")
        control_box.grid(row=2, column=0, sticky="ew", padx=10, pady=6)
        for idx in range(8):
            control_box.columnconfigure(idx, weight=1)
        ttk.Label(control_box, text="XYZ step").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(control_box, textvariable=self.step_var, width=8).grid(row=0, column=1, sticky="ew", padx=6, pady=6)
        ttk.Label(control_box, text="Angle step deg").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(control_box, textvariable=self.angle_step_deg_var, width=8).grid(row=0, column=3, sticky="ew", padx=6, pady=6)
        ttk.Button(control_box, text="Zero", command=self._zero).grid(row=0, column=4, sticky="ew", padx=6, pady=6)
        ttk.Button(control_box, text="Load", command=self._load).grid(row=0, column=5, sticky="ew", padx=6, pady=6)
        ttk.Button(control_box, text="Save", command=self._save).grid(row=0, column=6, sticky="ew", padx=6, pady=6)
        ttk.Label(control_box, textvariable=self.status_var).grid(row=0, column=7, sticky="ew", padx=6, pady=6)

    def _rad_to_deg_text(self, value: str) -> str:
        try:
            return "{:.6f}".format(math.degrees(float(value)))
        except Exception:
            return "0.0"

    def _apply_degrees(self, key: str) -> None:
        try:
            self.vars[key].set("{:.9f}".format(math.radians(float(self.deg_vars[key].get()))))
        except Exception as exc:
            messagebox.showerror("Invalid Angle", str(exc))

    def _nudge(self, key: str, sign: float) -> None:
        try:
            current = float(self.vars[key].get())
            if key in ("roll", "pitch", "yaw"):
                step = math.radians(float(self.angle_step_deg_var.get()))
            else:
                step = float(self.step_var.get())
            next_value = current + sign * step
            self.vars[key].set("{:.9f}".format(next_value))
            if key in self.deg_vars:
                self.deg_vars[key].set("{:.6f}".format(math.degrees(next_value)))
        except Exception as exc:
            messagebox.showerror("Invalid Step", str(exc))

    def _zero(self) -> None:
        for key in self.vars:
            self.vars[key].set("0.0")
        for key in self.deg_vars:
            self.deg_vars[key].set("0.0")

    def _snapshot(self) -> Dict[str, object]:
        data: Dict[str, object] = {
            "parent_frame": self.parent_var.get().strip(),
            "child_frame": self.child_var.get().strip(),
            "rate": self.rate_var.get().strip(),
        }
        for key, var in self.vars.items():
            data[key] = var.get().strip()
        return data

    def _apply_snapshot(self, data: Dict[str, object]) -> None:
        self.parent_var.set(str(data.get("parent_frame", self.parent_var.get())))
        self.child_var.set(str(data.get("child_frame", self.child_var.get())))
        self.rate_var.set(str(data.get("rate", self.rate_var.get())))
        for key, var in self.vars.items():
            if key in data:
                var.set(str(data[key]))
        for key in self.deg_vars:
            self.deg_vars[key].set(self._rad_to_deg_text(self.vars[key].get()))

    def _load(self) -> None:
        path = filedialog.askopenfilename(
            title="Load TF config",
            initialdir=str(Path.cwd() / "config"),
            filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("config must be a JSON object")
            self._apply_snapshot(data)
        except Exception as exc:
            messagebox.showerror("Load Failed", str(exc))

    def _save(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save TF config",
            initialdir=str(Path.cwd() / "config"),
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            Path(path).write_text(json.dumps(self._snapshot(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Save Failed", str(exc))

    def _schedule_publish(self) -> None:
        if rospy.is_shutdown() or self._shutdown:
            return

        delay_ms = 200
        try:
            rate_hz = max(1.0, float(self.rate_var.get()))
            delay_ms = max(10, int(1000.0 / rate_hz))
            self._publish_once()
        except Exception as exc:
            self.status_var.set("Invalid input")
            rospy.logwarn_throttle(2.0, "tf_adjust_ui: invalid transform input: %s", str(exc))

        self.root.after(delay_ms, self._schedule_publish)

    def _publish_once(self) -> None:
        parent_frame = self.parent_var.get().strip() or "map"
        child_frame = self.child_var.get().strip() or "world"
        x = float(self.vars["x"].get())
        y = float(self.vars["y"].get())
        z = float(self.vars["z"].get())
        roll = float(self.vars["roll"].get())
        pitch = float(self.vars["pitch"].get())
        yaw = float(self.vars["yaw"].get())
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(msg)
        self.status_var.set("{} -> {}".format(parent_frame, child_frame))

    def close(self) -> None:
        self._shutdown = True
        self.root.destroy()


def main() -> int:
    rospy.init_node("tf_adjust_ui", anonymous=True, disable_signals=True)
    root = tk.Tk()
    app = TfAdjustUI(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
