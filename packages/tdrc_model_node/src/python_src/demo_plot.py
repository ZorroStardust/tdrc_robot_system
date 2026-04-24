# -*- coding: utf-8 -*-
"""
demo_plot.py

Interactive demo for:
joint / cc / tendon / motor / recovered joint

Features
--------
- Tk sliders for joint parameters
- Real-time text display of:
    joint
    cc components
    tendon length changes
    motor angles
    recovered joint
    recovery errors
- Real-time bar plots for:
    tendon length changes (8 tendons)
    motor angles (4 motors)

Pure analytical demo, no MuJoCo dependency.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from model import (
    JointSpace,
    TDRCJointMotorModel,
    angle_diff,
)


# =========================
# Config
# =========================

R_HOLE = 0.003
D_SPOOL = 0.012

THETA_A_MAX_DEG = 180.0
THETA_C_MAX_DEG = 90.0

UPDATE_MS = 50

# Mapping: ideal motor index -> real motor index.
# Example for swapping physical motor 1/2 wiring:
# {1: 2, 2: 1, 3: 3, 4: 4}
MOTOR_INDEX_MAP = {1: 2, 2: 1, 3: 3, 4: 4}

# Direction map: real motor index -> direction sign.
# +1 means same as model direction, -1 means opposite z-axis direction.
# Example: real motor 1/3 are reversed.
# {1: -1, 2: 1, 3: -1, 4: 1}
MOTOR_DIRECTION_MAP = {1: -1, 2: -1, 3: -1, 4: -1}


# =========================
# Formatting helpers
# =========================

def fmt_float(x: float, digits: int = 6) -> str:
    return f"{x:.{digits}f}"

def fmt_deg(x_rad: float, digits: int = 3) -> str:
    return f"{math.degrees(x_rad):.{digits}f} deg"

def fmt_rad(x_rad: float, digits: int = 6) -> str:
    return f"{x_rad:.{digits}f} rad"

def fmt_err(x: float, digits: int = 3) -> str:
    return f"{x:.{digits}e}"

def set_text(widget: tk.Text, content: str):
    widget.config(state="normal")
    widget.delete("1.0", tk.END)
    widget.insert(tk.END, content)
    widget.config(state="disabled")


# =========================
# Main app
# =========================

class DemoApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TDRC Joint / CC / Tendon / Motor Demo")
        self.root.geometry("1500x900")
        self._ui_ready = False

        self.model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            motor_index_map=MOTOR_INDEX_MAP,
            motor_direction_map=MOTOR_DIRECTION_MAP,
        )

        self._build_ui()
        self._ui_ready = True
        self._schedule_update()

    # -------------------------
    # UI
    # -------------------------

    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main, padding=10)
        left.pack(side="left", fill="y")

        right = ttk.Frame(main, padding=10)
        right.pack(side="left", fill="both", expand=True)

        self._build_controls(left)
        self._build_text_panels(right)
        self._build_plots(right)

    def _build_controls(self, parent):
        box = ttk.LabelFrame(parent, text="Joint Controls", padding=10)
        box.pack(fill="x", padx=5, pady=5)

        self.phi_a_deg = tk.DoubleVar(value=0.0)
        self.theta_a_deg = tk.DoubleVar(value=0.0)
        self.phi_c_deg = tk.DoubleVar(value=0.0)
        self.theta_c_deg = tk.DoubleVar(value=0.0)

        self.auto_update = tk.BooleanVar(value=True)

        self._add_slider(box, "phi_a", self.phi_a_deg, -180.0, 180.0, 0.5, "deg")
        self._add_slider(box, "theta_a", self.theta_a_deg, 0.0, THETA_A_MAX_DEG, 0.5, "deg")
        self._add_slider(box, "phi_c", self.phi_c_deg, -180.0, 180.0, 0.5, "deg")
        self._add_slider(box, "theta_c", self.theta_c_deg, 0.0, THETA_C_MAX_DEG, 0.5, "deg")

        btn_row = ttk.Frame(box)
        btn_row.pack(fill="x", pady=(10, 0))

        ttk.Checkbutton(
            btn_row,
            text="Auto update",
            variable=self.auto_update,
        ).pack(side="left")

        ttk.Button(
            btn_row,
            text="Update now",
            command=self.update_view,
        ).pack(side="left", padx=8)

        ttk.Button(
            btn_row,
            text="Reset",
            command=self.reset_values,
        ).pack(side="left", padx=8)

        info_box = ttk.LabelFrame(parent, text="Model Parameters", padding=10)
        info_box.pack(fill="x", padx=5, pady=5)

        info = (
            f"r_hole   = {R_HOLE:.6f} m\n"
            f"d_spool  = {D_SPOOL:.6f} m\n"
            f"K = 2r/d = {self.model.K:.6f}\n\n"
            f"motor map (ideal->real): {self.model.motor_index_map}\n\n"
            f"motor dir (real->sign): {self.model.motor_direction_map}\n\n"
            f"theta_a range: [0, {THETA_A_MAX_DEG:.1f}] deg\n"
            f"theta_c range: [0, {THETA_C_MAX_DEG:.1f}] deg"
        )
        ttk.Label(info_box, text=info, justify="left").pack(anchor="w")

    def _add_slider(self, parent, label, var, frm, to, resolution, unit):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=6)

        ttk.Label(row, text=label, width=10).pack(side="left")

        scale = tk.Scale(
            row,
            from_=frm,
            to=to,
            orient="horizontal",
            resolution=resolution,
            variable=var,
            length=280,
        )
        scale.pack(side="left")

        value_label = ttk.Label(row, width=12, anchor="e")
        value_label.pack(side="left", padx=(8, 0))

        def _refresh_label(*_):
            value_label.config(text=f"{var.get():.2f} {unit}")
            if self._ui_ready and self.auto_update.get():
                self.update_view()

        var.trace_add("write", _refresh_label)
        _refresh_label()

    def _build_text_panels(self, parent):
        top = ttk.Frame(parent)
        top.pack(fill="x", expand=False)

        mid = ttk.Frame(parent)
        mid.pack(fill="x", expand=False, pady=(8, 0))

        bot = ttk.Frame(parent)
        bot.pack(fill="x", expand=False, pady=(8, 0))

        self.text_joint = self._make_text_panel(top, "Joint / Recovered Joint", width=58, height=16)
        self.text_cc = self._make_text_panel(top, "CC Components", width=42, height=16)

        self.text_tendon = self._make_text_panel(mid, "Tendon Length Changes", width=58, height=16)
        self.text_motor = self._make_text_panel(mid, "Motor Angles", width=42, height=16)

        self.text_error = self._make_text_panel(bot, "Recovery Errors / Flags", width=104, height=10)

    def _make_text_panel(self, parent, title, width=50, height=12):
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

        self.ax_tendon = self.fig.add_subplot(121)
        self.ax_motor = self.fig.add_subplot(122)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # -------------------------
    # Data
    # -------------------------

    def get_joint(self) -> JointSpace:
        return JointSpace(
            phi_a=math.radians(self.phi_a_deg.get()),
            theta_a=math.radians(self.theta_a_deg.get()),
            phi_c=math.radians(self.phi_c_deg.get()),
            theta_c=math.radians(self.theta_c_deg.get()),
        )

    def reset_values(self):
        self.phi_a_deg.set(0.0)
        self.theta_a_deg.set(0.0)
        self.phi_c_deg.set(0.0)
        self.theta_c_deg.set(0.0)
        self.update_view()

    # -------------------------
    # Update
    # -------------------------

    def update_view(self):
        joint = self.get_joint()

        cc = self.model.joint_to_cc_components(joint)
        tendon = self.model.joint_to_tendon_lengths(joint)
        motor = self.model.joint_to_motor_angles(joint)
        recovered = self.model.motor_angles_to_joint(motor)

        err_theta_a = recovered.theta_a - joint.theta_a
        err_theta_c = recovered.theta_c - joint.theta_c

        if recovered.singular_a:
            err_phi_a = float("nan")
        else:
            err_phi_a = angle_diff(recovered.phi_a, joint.phi_a)

        if recovered.singular_c:
            err_phi_c = float("nan")
        else:
            err_phi_c = angle_diff(recovered.phi_c, joint.phi_c)

        # Text blocks
        joint_text = (
            "[Original Joint]\n"
            f"phi_a   = {fmt_rad(joint.phi_a)}   ({fmt_deg(joint.phi_a)})\n"
            f"theta_a = {fmt_rad(joint.theta_a)}   ({fmt_deg(joint.theta_a)})\n"
            f"phi_c   = {fmt_rad(joint.phi_c)}   ({fmt_deg(joint.phi_c)})\n"
            f"theta_c = {fmt_rad(joint.theta_c)}   ({fmt_deg(joint.theta_c)})\n\n"
            "[Recovered Joint]\n"
            f"phi_a   = {fmt_rad(recovered.phi_a)}   ({fmt_deg(recovered.phi_a)})\n"
            f"theta_a = {fmt_rad(recovered.theta_a)}   ({fmt_deg(recovered.theta_a)})\n"
            f"phi_c   = {fmt_rad(recovered.phi_c)}   ({fmt_deg(recovered.phi_c)})\n"
            f"theta_c = {fmt_rad(recovered.theta_c)}   ({fmt_deg(recovered.theta_c)})"
        )
        set_text(self.text_joint, joint_text)

        cc_text = (
            f"u_ax = {fmt_float(cc.u_ax)}\n"
            f"u_ay = {fmt_float(cc.u_ay)}\n"
            f"u_cx = {fmt_float(cc.u_cx)}\n"
            f"u_cy = {fmt_float(cc.u_cy)}\n\n"
            "[Near segment relation]\n"
            f"alpha1 ?= K*u_ax = {fmt_float(self.model.K * cc.u_ax)}\n"
            f"alpha2 ?= K*u_ay = {fmt_float(self.model.K * cc.u_ay)}"
        )
        set_text(self.text_cc, cc_text)

        tendon_text = (
            f"dl1 = {fmt_float(tendon.dl1)}\n"
            f"dl2 = {fmt_float(tendon.dl2)}\n"
            f"dl3 = {fmt_float(tendon.dl3)}\n"
            f"dl4 = {fmt_float(tendon.dl4)}\n"
            f"dl5 = {fmt_float(tendon.dl5)}\n"
            f"dl6 = {fmt_float(tendon.dl6)}\n"
            f"dl7 = {fmt_float(tendon.dl7)}\n"
            f"dl8 = {fmt_float(tendon.dl8)}\n\n"
            "[Symmetry check]\n"
            f"dl3 + dl1 = {fmt_err(tendon.dl3 + tendon.dl1)}\n"
            f"dl4 + dl2 = {fmt_err(tendon.dl4 + tendon.dl2)}\n"
            f"dl7 + dl5 = {fmt_err(tendon.dl7 + tendon.dl5)}\n"
            f"dl8 + dl6 = {fmt_err(tendon.dl8 + tendon.dl6)}"
        )
        set_text(self.text_tendon, tendon_text)

        motor_text = (
            f"alpha1 = {fmt_float(motor.alpha1)} rad   ({fmt_deg(motor.alpha1)})\n"
            f"alpha2 = {fmt_float(motor.alpha2)} rad   ({fmt_deg(motor.alpha2)})\n"
            f"alpha3 = {fmt_float(motor.alpha3)} rad   ({fmt_deg(motor.alpha3)})\n"
            f"alpha4 = {fmt_float(motor.alpha4)} rad   ({fmt_deg(motor.alpha4)})\n\n"
            "[Direct consistency]\n"
            f"alpha1 - K*u_ax = {fmt_err(motor.alpha1 - self.model.K * cc.u_ax)}\n"
            f"alpha2 - K*u_ay = {fmt_err(motor.alpha2 - self.model.K * cc.u_ay)}"
        )
        set_text(self.text_motor, motor_text)

        err_phi_a_str = "N/A (singular)" if recovered.singular_a else f"{fmt_err(err_phi_a)} rad"
        err_phi_c_str = "N/A (singular)" if recovered.singular_c else f"{fmt_err(err_phi_c)} rad"

        error_text = (
            "[Recovery errors]\n"
            f"theta_a error = {fmt_err(err_theta_a)} rad\n"
            f"phi_a error   = {err_phi_a_str}\n"
            f"theta_c error = {fmt_err(err_theta_c)} rad\n"
            f"phi_c error   = {err_phi_c_str}\n\n"
            "[Singularity flags]\n"
            f"singular_a = {recovered.singular_a}\n"
            f"singular_c = {recovered.singular_c}\n\n"
            "[Interpretation]\n"
            "When theta_a or theta_c is near zero, the corresponding phi becomes\n"
            "physically undefined. The current model returns phi = 0 by convention\n"
            "and raises the singular flag."
        )
        set_text(self.text_error, error_text)

        self._update_plots(tendon, motor)

    def _update_plots(self, tendon, motor):
        self.ax_tendon.clear()
        self.ax_motor.clear()

        tendon_vals = np.array(tendon.as_tuple(), dtype=float)
        motor_vals = np.array(motor.as_tuple(), dtype=float)

        tendon_names = [f"dl{i}" for i in range(1, 9)]
        motor_names = [f"a{i}" for i in range(1, 5)]

        self.ax_tendon.bar(tendon_names, tendon_vals)
        self.ax_tendon.set_title("Tendon Length Changes")
        self.ax_tendon.set_ylabel("Length change")
        self.ax_tendon.grid(True, axis="y")

        self.ax_motor.bar(motor_names, motor_vals)
        self.ax_motor.set_title("Motor Angles")
        self.ax_motor.set_ylabel("Angle [rad]")
        self.ax_motor.grid(True, axis="y")

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _schedule_update(self):
        if self.auto_update.get():
            self.update_view()
        self.root.after(UPDATE_MS, self._schedule_update)


# =========================
# Main
# =========================

def main():
    root = tk.Tk()
    app = DemoApp(root)
    app.update_view()
    root.mainloop()


if __name__ == "__main__":
    main()
