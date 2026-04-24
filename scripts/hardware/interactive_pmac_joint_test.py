# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import queue
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "packages/tdrc_model_node/src/python_src"))
sys.path.insert(0, str(REPO_ROOT / "packages/pmac_sdk/src"))

from model import JointSpace, TDRCJointMotorModel
from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController


# =========================
# Config
# =========================

PMAC_IP = "192.168.0.200"

R_HOLE =  0.00215 # 2.15 mm 
D_SPOOL = 0.012   # 12 mm

THETA_A_MAX_DEG = 180.0
THETA_C_MAX_DEG = 90.0

CALIB_DELTA_LIMIT_DEG = 30.0
JOINT_MOVE_TIME_MS = 100
CALIB_MOVE_TIME_MS = 100
ACCEL = 50
SCURVE = 0

MOTOR_INDEX_MAP = {1: 2, 2: 1, 3: 3, 4: 4}
MOTOR_DIRECTION_MAP = {1: -1, 2: -1, 3: -1, 4: -1}


def rad_to_deg(x: float) -> float:
    return math.degrees(x)


def fmt_list(vals, digits=3):
    return "[" + ", ".join(f"{v:.{digits}f}" for v in vals) + "]"


class InteractivePMACJointTestApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TDRC PMAC Interactive Flow Test")
        self.root.geometry("1500x1050")
        self.root.minsize(1500, 950)

        self.log_queue: queue.Queue[str] = queue.Queue()

        self.model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            motor_index_map=MOTOR_INDEX_MAP,
            motor_direction_map=MOTOR_DIRECTION_MAP,
        )

        self.config = PMACConfig(ip=PMAC_IP)
        self.robot: PMACRobotController | None = None

        self.connected = False
        self.zero_ready = False
        self.control_mode = "calibration"
        self.control_mode = "calibration"  # "calibration" or "joint_control"

        self.calibration_base_pulses = [0, 0, 0, 0, 0]
        self.zero_pulses = [0, 0, 0, 0, 0]
        self.last_target_pulses = [0, 0, 0, 0, 0]

        self._ui_ready = False
        self._build_ui()
        self._ui_ready = True

        self.root.after(100, self._process_logs)

    # =========================
    # UI
    # =========================

    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main, padding=10)
        left.pack(side="left", fill="y")

        right = ttk.Frame(main, padding=10)
        right.pack(side="left", fill="both", expand=True)

        self._build_connection_panel(left)
        self._build_calibration_panel(left)
        self._build_joint_panel(left)
        self._build_status_panel(right)
        self._build_plots(right)

    def _build_connection_panel(self, parent):
        box = ttk.LabelFrame(parent, text="1. PMAC Connection", padding=10)
        box.pack(fill="x", padx=5, pady=5)

        self.ip_var = tk.StringVar(value=PMAC_IP)
        self.boot_var = tk.BooleanVar(value=False)

        row = ttk.Frame(box)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="PMAC IP", width=12).pack(side="left")
        ttk.Entry(row, textvariable=self.ip_var, width=18).pack(side="left")

        ttk.Checkbutton(
            box,
            text="Run hardware_boot before Modbus connect",
            variable=self.boot_var,
        ).pack(anchor="w", pady=4)

        ttk.Button(
            box,
            text="Connect PMAC",
            command=self.connect_pmac_threaded,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Read Current Pulses",
            command=self.read_current_pulses,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Close PMAC",
            command=self.close_pmac,
        ).pack(fill="x", pady=4)

    def _build_calibration_panel(self, parent):
        box = ttk.LabelFrame(parent, text="2. Manual Motor Calibration", padding=10)
        box.pack(fill="x", padx=5, pady=5)

        self.calib_live = tk.BooleanVar(value=False)
        self.calib_delta_deg = [tk.DoubleVar(value=0.0) for _ in range(4)]

        ttk.Checkbutton(
            box,
            text="Live send calibration deltas",
            variable=self.calib_live,
        ).pack(anchor="w", pady=(0, 8))

        for i, var in enumerate(self.calib_delta_deg):
            self._add_slider(
                box,
                label=f"motor {i+1} delta",
                var=var,
                frm=-CALIB_DELTA_LIMIT_DEG,
                to=CALIB_DELTA_LIMIT_DEG,
                resolution=0.1,
                unit="deg",
                callback=self._on_calib_slider_changed,
            )

        ttk.Button(
            box,
            text="Send Calibration Deltas Once",
            command=self.send_calibration_delta,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Set Current As Zero",
            command=self.set_current_as_zero,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Return To Calibration Mode",
            command=self.return_to_calibration_mode,
        ).pack(fill="x", pady=4)

    def _build_joint_panel(self, parent):
        box = ttk.LabelFrame(parent, text="3. Joint Space Control", padding=10)
        box.pack(fill="x", padx=5, pady=5)

        self.joint_live = tk.BooleanVar(value=False)

        self.phi_a_deg = tk.DoubleVar(value=0.0)
        self.theta_a_deg = tk.DoubleVar(value=0.0)
        self.phi_c_deg = tk.DoubleVar(value=0.0)
        self.theta_c_deg = tk.DoubleVar(value=0.0)

        ttk.Checkbutton(
            box,
            text="Live send joint target",
            variable=self.joint_live,
        ).pack(anchor="w", pady=(0, 8))

        self._add_slider(box, "phi_a", self.phi_a_deg, -180.0, 180.0, 0.5, "deg", self._on_joint_slider_changed)
        self._add_slider(box, "theta_a", self.theta_a_deg, 0.0, THETA_A_MAX_DEG, 0.5, "deg", self._on_joint_slider_changed)
        self._add_slider(box, "phi_c", self.phi_c_deg, -180.0, 180.0, 0.5, "deg", self._on_joint_slider_changed)
        self._add_slider(box, "theta_c", self.theta_c_deg, 0.0, THETA_C_MAX_DEG, 0.5, "deg", self._on_joint_slider_changed)

        ttk.Button(
            box,
            text="Send Joint Target Once",
            command=self.send_joint_target,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Reset Joint Sliders",
            command=self.reset_joint_sliders,
        ).pack(fill="x", pady=4)

        ttk.Button(
            box,
            text="Enter Joint Control Mode",
            command=self.enter_joint_control_mode,
        ).pack(fill="x", pady=4)

    def _add_slider(self, parent, label, var, frm, to, resolution, unit, callback):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=4)

        ttk.Label(row, text=label, width=14).pack(side="left")

        scale = tk.Scale(
            row,
            from_=frm,
            to=to,
            orient="horizontal",
            resolution=resolution,
            variable=var,
            length=260,
        )
        scale.pack(side="left")

        value_label = ttk.Label(row, width=12, anchor="e")
        value_label.pack(side="left", padx=(8, 0))

        def refresh(*_):
            value_label.config(text=f"{var.get():.2f} {unit}")
            if self._ui_ready:
                callback()

        var.trace_add("write", refresh)
        refresh()

    def _build_status_panel(self, parent):
        top = ttk.Frame(parent)
        top.pack(fill="both", expand=True)

        self.text_status = self._make_text_panel(top, "Status / Logs", width=80, height=22)
        self.text_calc = self._make_text_panel(top, "Current Calculation", width=70, height=22)

    def _make_text_panel(self, parent, title, width, height):
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.pack(side="left", fill="both", expand=True, padx=4)

        text = tk.Text(frame, width=width, height=height, wrap="word")
        text.pack(fill="both", expand=True)
        text.config(state="disabled")
        return text

    def _build_plots(self, parent):
        plot_frame = ttk.Frame(parent)
        plot_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.fig = plt.Figure(figsize=(10, 4.8), dpi=100)
        self.ax_motor_rad = self.fig.add_subplot(121)
        self.ax_pulses = self.fig.add_subplot(122)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # =========================
    # PMAC
    # =========================

    def connect_pmac_threaded(self):
        t = threading.Thread(target=self.connect_pmac, daemon=True)
        t.start()

    def connect_pmac(self):
        try:
            self._log("Connecting PMAC...")

            self.config = PMACConfig(ip=self.ip_var.get())
            self.robot = PMACRobotController(self.config)

            if self.boot_var.get():
                self._log("Running hardware_boot...")
                self.robot.hardware_boot()
                time.sleep(2.0)

            self.robot.connect_and_home()

            self.connected = True
            self.calibration_base_pulses = list(self.robot.base_positions)
            self.zero_pulses = list(self.robot.base_positions)
            self.last_target_pulses = list(self.robot.base_positions)

            self._log(f"PMAC connected.")
            self._log(f"Calibration base pulses: {self.calibration_base_pulses}")
            self._log("Now use motor delta sliders to manually straighten the continuum.")

        except Exception as e:
            self.connected = False
            self._log(f"ERROR: failed to connect PMAC: {e}")

    def close_pmac(self):
        try:
            if self.robot is not None:
                self.robot.close()
            self.connected = False
            self.zero_ready = False
            self._log("PMAC connection closed.")
        except Exception as e:
            self._log(f"ERROR: close failed: {e}")

    def read_current_pulses(self):
        if not self._check_connected():
            return
        try:
            current = self.robot.modbus.read_int32_array(address=10, count=5)
            self._log(f"Current pulses: {current}")
        except Exception as e:
            self._log(f"ERROR: read current pulses failed: {e}")

    def _check_connected(self) -> bool:
        if not self.connected or self.robot is None:
            messagebox.showwarning("PMAC not connected", "Please connect PMAC first.")
            return False
        return True

    # =========================
    # Calibration
    # =========================

    def _on_calib_slider_changed(self):
        if self.calib_live.get() and self.connected and self.control_mode == "calibration": 
            self.send_calibration_delta()

    def send_calibration_delta(self):
        if not self._check_connected():
            return

        if self.control_mode != "calibration":
            self._log("Not in calibration mode. Ignore calibration delta command.")
            return

        deltas_deg = [v.get() for v in self.calib_delta_deg]
        target = list(self.calibration_base_pulses)

        for i in range(4):
            target[i] = int(
                self.calibration_base_pulses[i]
                + deltas_deg[i] * self.config.pulses_per_degree
            )

        self._send_pmac_target(
            target,
            move_time=CALIB_MOVE_TIME_MS,
            accel=ACCEL,
            scurve=SCURVE,
            tag="calibration_delta",
        )

        self._update_calc_panel(
            motor_rad=[math.radians(x) for x in deltas_deg],
            target_pulses=target,
            extra="Calibration delta mode\n"
                  "These deltas are relative to calibration_base_pulses.\n"
                  "After the continuum is roughly straight, click Set Current As Zero."
        )

    def set_current_as_zero(self):
        if not self._check_connected():
            return

        try:
            current = self.robot.modbus.read_int32_array(address=10, count=5)

            self.zero_pulses = list(current)
            self.robot.config.zero_offsets = list(current)
            self.robot.base_positions = list(current)

            self.zero_ready = True
            self.control_mode = "joint"

            for var in self.calib_delta_deg:
                var.set(0.0)

            self._log("Current physical position has been set as zero.")
            self._log(f"Zero pulses: {self.zero_pulses}")
            self._log("Joint-space control is now enabled.")

        except Exception as e:
            self._log(f"ERROR: set current as zero failed: {e}")
    
    def return_to_calibration_mode(self):
        if not self._check_connected():
            return

        self.joint_live.set(False)
        self.calib_live.set(False)

        try:
            current = self.robot.modbus.read_int32_array(address=10, count=5)
            self.calibration_base_pulses = list(current)

            for var in self.calib_delta_deg:
                var.set(0.0)

            self.control_mode = "calibration"
            self.zero_ready = False

            self._log("Returned to calibration mode.")
            self._log(f"New calibration base pulses: {self.calibration_base_pulses}")
            self._log("Use motor delta sliders to re-straighten the continuum, then click Set Current As Zero.")

        except Exception as e:
            self._log(f"ERROR: return to calibration mode failed: {e}")


    def enter_joint_mode(self):
        if not self._check_connected():
            return

        if not self.zero_ready:
            messagebox.showwarning(
                "Zero not ready",
                "Please click Set Current As Zero before entering joint control mode.",
            )
            return

        self.calib_live.set(False)
        self.control_mode = "joint"

        self._log("Entered joint control mode.")

    # Backward-compatible name used by button bindings.
    def enter_joint_control_mode(self):
        self.enter_joint_mode()


    # =========================
    # Joint control
    # =========================

    def _on_joint_slider_changed(self):
        self.update_joint_preview()
        if (
        self.joint_live.get()
        and self.connected
        and self.zero_ready
        and self.control_mode == "joint"
        ):
            self.send_joint_target()


    def get_joint(self) -> JointSpace:
        return JointSpace(
            phi_a=math.radians(self.phi_a_deg.get()),
            theta_a=math.radians(self.theta_a_deg.get()),
            phi_c=math.radians(self.phi_c_deg.get()),
            theta_c=math.radians(self.theta_c_deg.get()),
        )

    def reset_joint_sliders(self):
        self.phi_a_deg.set(0.0)
        self.theta_a_deg.set(0.0)
        self.phi_c_deg.set(0.0)
        self.theta_c_deg.set(0.0)
        self.update_joint_preview()

    def update_joint_preview(self):
        joint = self.get_joint()
        motor = self.model.joint_to_motor_angles(joint)
        motor_rad = list(motor.as_tuple())

        target_pulses = self.motor_rad_to_absolute_pulses(motor_rad)

        self._update_calc_panel(
            motor_rad=motor_rad,
            target_pulses=target_pulses,
            extra="Joint-space preview only.\n"
                  "Target pulses are computed from zero_pulses + motor_angle_deg * pulses_per_degree."
        )

    def send_joint_target(self):
        if not self._check_connected():
            return

        if not self.zero_ready:
            messagebox.showwarning(
                "Zero not ready",
                "Please finish manual calibration and click Set Current As Zero first.",
            )
        if self.control_mode != "joint":
            messagebox.showwarning(
                "Not in joint mode",
                "Please click Enter Joint Control Mode first.",
            )
            return

        joint = self.get_joint()
        motor = self.model.joint_to_motor_angles(joint)
        motor_rad = list(motor.as_tuple())

        target_pulses = self.motor_rad_to_absolute_pulses(motor_rad)

        self._send_pmac_target(
            target_pulses,
            move_time=JOINT_MOVE_TIME_MS,
            accel=ACCEL,
            scurve=SCURVE,
            tag="joint_target",
        )

        self._update_calc_panel(
            motor_rad=motor_rad,
            target_pulses=target_pulses,
            extra="Joint target sent to PMAC."
        )

    def motor_rad_to_absolute_pulses(self, motor_rad: list[float]) -> list[int]:
        target = list(self.zero_pulses)

        for i in range(4):
            angle_deg = rad_to_deg(motor_rad[i])
            target[i] = int(
                self.zero_pulses[i]
                + angle_deg * self.config.pulses_per_degree
            )

        return target

    # =========================
    # Low-level send
    # =========================

    def _send_pmac_target(self, target_pulses, move_time, accel, scurve, tag):
        try:
            self.robot.move_joints(
                target_pulses=target_pulses,
                move_time=move_time,
                accel=accel,
                scurve=scurve,
            )
            self.last_target_pulses = list(target_pulses)
            self._log(f"SEND [{tag}] target_pulses={target_pulses}, move_time={move_time}ms")
            self._update_plots()
        except Exception as e:
            self._log(f"ERROR: send PMAC target failed: {e}")

    # =========================
    # Display
    # =========================

    def _update_calc_panel(self, motor_rad, target_pulses, extra=""):
        joint = self.get_joint()

        text = (
            "[Joint]\n"
            f"phi_a   = {self.phi_a_deg.get():.3f} deg\n"
            f"theta_a = {self.theta_a_deg.get():.3f} deg\n"
            f"phi_c   = {self.phi_c_deg.get():.3f} deg\n"
            f"theta_c = {self.theta_c_deg.get():.3f} deg\n\n"
            "[Motor angles]\n"
            f"rad = {fmt_list(motor_rad, 6)}\n"
            f"deg = {fmt_list([rad_to_deg(x) for x in motor_rad], 3)}\n\n"
            "[Pulses]\n"
            f"zero_pulses   = {self.zero_pulses}\n"
            f"target_pulses = {target_pulses}\n\n"
            f"{extra}\n"
        )

        self._set_text(self.text_calc, text)
        self._update_plots()

    def _update_plots(self):
        self.ax_motor_rad.clear()
        self.ax_pulses.clear()

        joint = self.get_joint()
        motor = self.model.joint_to_motor_angles(joint)
        motor_rad = np.array(motor.as_tuple(), dtype=float)

        pulse_delta = np.array(self.last_target_pulses, dtype=float) - np.array(self.zero_pulses, dtype=float)

        self.ax_motor_rad.bar([f"m{i}" for i in range(1, 5)], motor_rad)
        self.ax_motor_rad.set_title("Motor Angles")
        self.ax_motor_rad.set_ylabel("rad")
        self.ax_motor_rad.grid(True, axis="y")

        self.ax_pulses.bar([f"axis{i}" for i in range(1, 6)], pulse_delta)
        self.ax_pulses.set_title("Pulse Delta From Zero")
        self.ax_pulses.set_ylabel("pulses")
        self.ax_pulses.grid(True, axis="y")

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _log(self, msg: str):
        self.log_queue.put(msg)

    def _process_logs(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                old = self.text_status.get("1.0", tk.END)
                new = old + f"[{time.strftime('%H:%M:%S')}] {msg}\n"
                self._set_text(self.text_status, new)
        except queue.Empty:
            pass

        self.root.after(100, self._process_logs)

    @staticmethod
    def _set_text(widget: tk.Text, content: str):
        widget.config(state="normal")
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, content)
        widget.see(tk.END)
        widget.config(state="disabled")


def main():
    root = tk.Tk()
    app = InteractivePMACJointTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
