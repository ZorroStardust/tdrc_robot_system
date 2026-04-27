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

R_HOLE = 0.00215  # 2.15 mm
D_SPOOL = 0.012   # 12 mm

THETA_A_MAX_DEG = 180.0
THETA_C_MAX_DEG = 90.0

CALIB_DELTA_LIMIT_DEG = 180.0
JOINT_MOVE_TIME_MS = 100
CALIB_MOVE_TIME_MS = 100
ACCEL = 50
SCURVE = 0

# ideal motor index -> real PMAC axis index
MOTOR_INDEX_MAP = {1: 2, 2: 1, 3: 3, 4: 4}
# MOTOR_INDEX_MAP = {1: 1, 2: 2, 3: 3, 4: 4}

# real PMAC axis index -> direction sign
MOTOR_DIRECTION_MAP = {1: -1, 2: 1, 3: -1, 4: -1}
# MOTOR_DIRECTION_MAP = {1: 1, 2: 1, 3: 1, 4: 1}


def rad_to_deg(x: float) -> float:
    return math.degrees(x)


def fmt_list(vals, digits=3):
    return "[" + ", ".join(f"{v:.{digits}f}" for v in vals) + "]"


class InteractivePMACJointTestApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TDRC PMAC Interactive Flow Test")
        self.root.geometry("1500x950")
        self.root.minsize(1200, 800)

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
        self.control_mode = "calibration"  # "calibration" or "joint"

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

        left = self._build_scrollable_left_panel(main)

        right = ttk.Frame(main, padding=10)
        right.pack(side="left", fill="both", expand=True)

        self._build_connection_panel(left)
        self._build_calibration_panel(left)
        self._build_joint_panel(left)
        self._build_status_panel(right)
        self._build_plots(right)

    def _build_scrollable_left_panel(self, parent):
        left_container = ttk.Frame(parent)
        left_container.pack(side="left", fill="y")

        left_canvas = tk.Canvas(left_container, width=560, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(
            left_container,
            orient="vertical",
            command=left_canvas.yview,
        )
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_scrollbar.pack(side="right", fill="y")
        left_canvas.pack(side="left", fill="both", expand=True)

        left = ttk.Frame(left_canvas, padding=10)
        left_window = left_canvas.create_window((0, 0), window=left, anchor="nw")

        def _on_left_configure(_event):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))

        def _on_canvas_configure(event):
            left_canvas.itemconfigure(left_window, width=event.width)

        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        left.bind("<Configure>", _on_left_configure)
        left_canvas.bind("<Configure>", _on_canvas_configure)
        left_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        return left

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
        box = ttk.LabelFrame(
            parent,
            text="2. Manual Motor Calibration",
            padding=10,
        )
        box.pack(fill="x", padx=5, pady=5)

        self.calib_live = tk.BooleanVar(value=False)
        self.calib_axis_mode = tk.StringVar(value="real")  # "real" or "ideal"
        self.calib_delta_deg = [tk.DoubleVar(value=0.0) for _ in range(4)]

        mode_box = ttk.LabelFrame(box, text="Calibration Axis Mode", padding=6)
        mode_box.pack(fill="x", pady=(0, 8))

        ttk.Radiobutton(
            mode_box,
            text="Real PMAC Axis",
            variable=self.calib_axis_mode,
            value="real",
            command=self._on_calib_axis_mode_changed,
        ).pack(anchor="w")

        ttk.Radiobutton(
            mode_box,
            text="Ideal Model Motor",
            variable=self.calib_axis_mode,
            value="ideal",
            command=self._on_calib_axis_mode_changed,
        ).pack(anchor="w")

        self.mapping_label = ttk.Label(
            mode_box,
            text=self._mapping_text(),
            justify="left",
        )
        self.mapping_label.pack(anchor="w", pady=(6, 0))

        ttk.Checkbutton(
            box,
            text="Live send calibration deltas",
            variable=self.calib_live,
        ).pack(anchor="w", pady=(0, 8))

        self.calib_slider_labels: list[ttk.Label] = []
        for i, var in enumerate(self.calib_delta_deg):
            self._add_slider(
                box,
                label=self._calib_slider_label(i),
                var=var,
                frm=-CALIB_DELTA_LIMIT_DEG,
                to=CALIB_DELTA_LIMIT_DEG,
                resolution=0.1,
                unit="deg",
                callback=self._on_calib_slider_changed,
                label_store=self.calib_slider_labels,
            )

        self._build_single_axis_direct_test_panel(box)

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

    def _build_single_axis_direct_test_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Single Axis Direct Test", padding=6)
        box.pack(fill="x", pady=(8, 8))

        self.direct_axis = tk.IntVar(value=1)
        self.direct_delta_deg = tk.DoubleVar(value=0.0)
        self.direct_abs_pulse = tk.IntVar(value=0)

        row = ttk.Frame(box)
        row.pack(fill="x", pady=3)

        ttk.Label(row, text="real axis", width=10).pack(side="left")
        ttk.Entry(row, textvariable=self.direct_axis, width=6).pack(side="left")

        ttk.Label(row, text="delta deg", width=10).pack(side="left", padx=(8, 0))
        ttk.Entry(row, textvariable=self.direct_delta_deg, width=10).pack(side="left")

        ttk.Button(
            row,
            text="Move Delta",
            command=self.move_real_axis_delta_input,
        ).pack(side="left", padx=(8, 0))

        row = ttk.Frame(box)
        row.pack(fill="x", pady=3)

        ttk.Label(row, text="real axis", width=10).pack(side="left")
        ttk.Entry(row, textvariable=self.direct_axis, width=6).pack(side="left")

        ttk.Label(row, text="abs pulse", width=10).pack(side="left", padx=(8, 0))
        ttk.Entry(row, textvariable=self.direct_abs_pulse, width=14).pack(side="left")

        ttk.Button(
            row,
            text="Move Abs Pulse",
            command=self.move_real_axis_abs_pulse_input,
        ).pack(side="left", padx=(8, 0))

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

    def _add_slider(
        self,
        parent,
        label,
        var,
        frm,
        to,
        resolution,
        unit,
        callback,
        label_store: list[ttk.Label] | None = None,
    ):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=4)

        name_label = ttk.Label(row, text=label, width=24)
        name_label.pack(side="left")

        if label_store is not None:
            label_store.append(name_label)

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
    # UI helpers
    # =========================

    def _calib_slider_label(self, idx: int) -> str:
        if self.calib_axis_mode.get() == "real":
            return f"real axis {idx + 1} delta"

        ideal_motor = idx + 1
        real_motor = MOTOR_INDEX_MAP[ideal_motor]
        sign = MOTOR_DIRECTION_MAP[real_motor]
        return f"ideal motor {ideal_motor} -> real {real_motor} ({sign:+d})"

    def _mapping_text(self) -> str:
        lines = ["Current ideal-to-real mapping:"]
        for ideal_motor in range(1, 5):
            real_motor = MOTOR_INDEX_MAP[ideal_motor]
            sign = MOTOR_DIRECTION_MAP[real_motor]
            lines.append(f"  ideal {ideal_motor} -> real {real_motor}, direction sign = {sign:+d}")
        return "\n".join(lines)

    def _on_calib_axis_mode_changed(self):
        if hasattr(self, "mapping_label"):
            self.mapping_label.config(text=self._mapping_text())

        if hasattr(self, "calib_slider_labels"):
            for i, label in enumerate(self.calib_slider_labels):
                label.config(text=self._calib_slider_label(i))

        mode = self.calib_axis_mode.get()
        self._log(f"Calibration axis mode changed to: {mode}")

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


            self._log("Synchronizing PMAC target to current position...")
            safe_pos = self.robot.hold_current_position()
            self._log(f"Hold current position written: {safe_pos}")

            self.connected = True
            self.control_mode = "calibration"
            self.zero_ready = False

            self.calibration_base_pulses = list(self.robot.base_positions)
            self.zero_pulses = list(self.robot.base_positions)
            self.last_target_pulses = list(self.robot.base_positions)

            self._log("PMAC connected.")
            self._log(f"Calibration base pulses: {self.calibration_base_pulses}")
            self._log("Now use calibration sliders to test real axis or ideal motor mapping.")

        except Exception as e:
            self.connected = False
            self._log(f"ERROR: failed to connect PMAC: {e}")

    def close_pmac(self):
        try:
            if self.robot is not None:
                self.robot.close()
            self.connected = False
            self.zero_ready = False
            self.control_mode = "calibration"
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

        mode = self.calib_axis_mode.get()

        for idx in range(4):
            delta_deg = deltas_deg[idx]

            if mode == "real":
                real_axis = idx + 1
                real_i = real_axis - 1
                sign = 1.0
            else:
                ideal_motor = idx + 1
                real_axis = MOTOR_INDEX_MAP[ideal_motor]
                real_i = real_axis - 1
                sign = float(MOTOR_DIRECTION_MAP[real_axis])

            target[real_i] = int(
                self.calibration_base_pulses[real_i]
                + sign * delta_deg * self.config.pulses_per_degree
            )

        self._send_pmac_target(
            target,
            move_time=CALIB_MOVE_TIME_MS,
            accel=ACCEL,
            scurve=SCURVE,
            tag=f"calibration_delta_{mode}",
        )

        self._update_calc_panel(
            motor_rad=[math.radians(x) for x in deltas_deg],
            target_pulses=target,
            extra=(
                f"Calibration delta mode: {mode}\n"
                "Real mode: slider i directly controls real PMAC axis i.\n"
                "Ideal mode: slider i means ideal model motor i and is mapped to real axis.\n"
                "After the continuum is roughly straight, click Set Current As Zero.\n"
            ),
        )

    def _get_direct_axis_index(self) -> int | None:
        axis = self.direct_axis.get()

        if axis < 1 or axis > 5:
            messagebox.showwarning(
                "Invalid axis",
                "Axis must be 1~5.",
            )
            return None

        return axis - 1

    def move_real_axis_delta_input(self):
        if not self._check_connected():
            return

        if self.control_mode != "calibration":
            messagebox.showwarning(
                "Not in calibration mode",
                "Please return to calibration mode first.",
            )
            return

        axis_i = self._get_direct_axis_index()
        if axis_i is None:
            return

        delta_deg = self.direct_delta_deg.get()

        target = list(self.calibration_base_pulses)
        target[axis_i] = int(
            self.calibration_base_pulses[axis_i]
            + delta_deg * self.config.pulses_per_degree
        )

        self._send_pmac_target(
            target,
            move_time=CALIB_MOVE_TIME_MS,
            accel=ACCEL,
            scurve=SCURVE,
            tag=f"real_axis_{axis_i + 1}_delta_input",
        )

        self._update_calc_panel(
            motor_rad=[0.0, 0.0, 0.0, 0.0],
            target_pulses=target,
            extra=(
                "Single real axis delta input.\n"
                f"real axis = {axis_i + 1}\n"
                f"delta_deg = {delta_deg:.3f}\n"
                "Delta is relative to calibration_base_pulses.\n"
            ),
        )

    def move_real_axis_abs_pulse_input(self):
        if not self._check_connected():
            return

        axis_i = self._get_direct_axis_index()
        if axis_i is None:
            return

        target = list(self.last_target_pulses)
        target[axis_i] = int(self.direct_abs_pulse.get())

        self._send_pmac_target(
            target,
            move_time=CALIB_MOVE_TIME_MS,
            accel=ACCEL,
            scurve=SCURVE,
            tag=f"real_axis_{axis_i + 1}_abs_pulse_input",
        )

        self._update_calc_panel(
            motor_rad=[0.0, 0.0, 0.0, 0.0],
            target_pulses=target,
            extra=(
                "Single real axis absolute pulse input.\n"
                f"real axis = {axis_i + 1}\n"
                f"abs_pulse = {self.direct_abs_pulse.get()}\n"
                "Target is based on last_target_pulses with only this axis replaced.\n"
            ),
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
            self._log("Use calibration sliders to re-straighten or test mapping, then click Set Current As Zero.")

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
            extra=(
                "Joint-space preview only.\n"
                "Target pulses are computed from zero_pulses + motor_angle_deg * pulses_per_degree.\n"
                "Joint-space motor angles already use model motor mapping internally.\n"
            ),
        )

    def send_joint_target(self):
        if not self._check_connected():
            return

        if not self.zero_ready:
            messagebox.showwarning(
                "Zero not ready",
                "Please finish manual calibration and click Set Current As Zero first.",
            )
            return

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
            extra="Joint target sent to PMAC.",
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
        text = (
            "[System]\n"
            f"control_mode = {self.control_mode}\n"
            f"calib_axis_mode = {getattr(self, 'calib_axis_mode', tk.StringVar(value='N/A')).get()}\n\n"
            "[Joint]\n"
            f"phi_a   = {self.phi_a_deg.get():.3f} deg\n"
            f"theta_a = {self.theta_a_deg.get():.3f} deg\n"
            f"phi_c   = {self.phi_c_deg.get():.3f} deg\n"
            f"theta_c = {self.theta_c_deg.get():.3f} deg\n\n"
            "[Motor angles shown]\n"
            f"rad = {fmt_list(motor_rad, 6)}\n"
            f"deg = {fmt_list([rad_to_deg(x) for x in motor_rad], 3)}\n\n"
            "[Pulses]\n"
            f"calibration_base_pulses = {self.calibration_base_pulses}\n"
            f"zero_pulses             = {self.zero_pulses}\n"
            f"target_pulses           = {target_pulses}\n\n"
            "[Mapping]\n"
            f"{self._mapping_text()}\n\n"
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
        self.ax_motor_rad.set_title("Joint Model Motor Angles")
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
