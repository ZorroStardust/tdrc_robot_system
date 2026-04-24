# -*- coding: utf-8 -*-
"""
test_compare_cc.py

Compare:
1) MuJoCo-style constant-curvature control components
   (u_ax, u_ay, u_cx, u_cy)
with
2) Real actuator motor angles
   (alpha1, alpha2, alpha3, alpha4)

Main goals:
- Verify alpha1/alpha2 are linearly proportional to u_ax/u_ay
- Show alpha3/alpha4 contain proximal-distal coupling
"""

from __future__ import annotations

import math
import random
from dataclasses import asdict

import numpy as np
import matplotlib.pyplot as plt

from model import JointSpace, TDRCJointMotorModel


# =========================
# Config
# =========================

R_HOLE = 0.003
D_SPOOL = 0.012

THETA_A_MAX = math.pi
THETA_C_MAX = math.pi / 2.0

N_RANDOM = 2000
SEED = 42


# =========================
# Sampling
# =========================

def sample_joint() -> JointSpace:
    return JointSpace(
        phi_a=random.uniform(-math.pi, math.pi),
        theta_a=random.uniform(0.0, THETA_A_MAX),
        phi_c=random.uniform(-math.pi, math.pi),
        theta_c=random.uniform(0.0, THETA_C_MAX),
    )


# =========================
# Main analysis
# =========================

def main():
    random.seed(SEED)

    model = TDRCJointMotorModel(
        hole_radius=R_HOLE,
        spool_diameter=D_SPOOL,
        cc_sign=-1.0,
    )

    K = model.K
    print(f"K = 2r/d = {K:.6f}")

    # Storage
    u_ax_list, u_ay_list, u_cx_list, u_cy_list = [], [], [], []
    alpha1_list, alpha2_list, alpha3_list, alpha4_list = [], [], [], []

    # distal motor decomposition
    alpha3_prox_list, alpha3_dist_list, alpha3_sum_list = [], [], []
    alpha4_prox_list, alpha4_dist_list, alpha4_sum_list = [], [], []

    # expected-vs-actual residuals
    alpha1_residual_list, alpha2_residual_list = [], []
    alpha3_coupling_residual_list, alpha4_coupling_residual_list = [], []

    for _ in range(N_RANDOM):
        joint = sample_joint()
        cc = model.joint_to_cc_components(joint)
        motor = model.joint_to_motor_angles(joint)

        # store cc
        u_ax_list.append(cc.u_ax)
        u_ay_list.append(cc.u_ay)
        u_cx_list.append(cc.u_cx)
        u_cy_list.append(cc.u_cy)

        # store motor
        alpha1_list.append(motor.alpha1)
        alpha2_list.append(motor.alpha2)
        alpha3_list.append(motor.alpha3)
        alpha4_list.append(motor.alpha4)

        # near segment expectation:
        # alpha1 = K * u_ax
        # alpha2 = K * u_ay
        alpha1_residual_list.append(motor.alpha1 - K * cc.u_ax)
        alpha2_residual_list.append(motor.alpha2 - K * cc.u_ay)

        # distal decomposition
        alpha3_prox = -K * joint.theta_a * math.cos(math.pi / 4.0 - joint.phi_a)
        alpha3_dist = -K * joint.theta_c * math.cos(math.pi / 4.0 - joint.phi_c)
        alpha3_sum = alpha3_prox + alpha3_dist

        alpha4_prox = -K * joint.theta_a * math.cos(3.0 * math.pi / 4.0 - joint.phi_a)
        alpha4_dist = -K * joint.theta_c * math.cos(3.0 * math.pi / 4.0 - joint.phi_c)
        alpha4_sum = alpha4_prox + alpha4_dist

        alpha3_prox_list.append(alpha3_prox)
        alpha3_dist_list.append(alpha3_dist)
        alpha3_sum_list.append(alpha3_sum)

        alpha4_prox_list.append(alpha4_prox)
        alpha4_dist_list.append(alpha4_dist)
        alpha4_sum_list.append(alpha4_sum)

        alpha3_coupling_residual_list.append(motor.alpha3 - alpha3_sum)
        alpha4_coupling_residual_list.append(motor.alpha4 - alpha4_sum)

    # convert to numpy
    u_ax = np.array(u_ax_list)
    u_ay = np.array(u_ay_list)
    u_cx = np.array(u_cx_list)
    u_cy = np.array(u_cy_list)

    alpha1 = np.array(alpha1_list)
    alpha2 = np.array(alpha2_list)
    alpha3 = np.array(alpha3_list)
    alpha4 = np.array(alpha4_list)

    alpha3_prox = np.array(alpha3_prox_list)
    alpha3_dist = np.array(alpha3_dist_list)
    alpha3_sum = np.array(alpha3_sum_list)

    alpha4_prox = np.array(alpha4_prox_list)
    alpha4_dist = np.array(alpha4_dist_list)
    alpha4_sum = np.array(alpha4_sum_list)

    alpha1_residual = np.array(alpha1_residual_list)
    alpha2_residual = np.array(alpha2_residual_list)
    alpha3_coupling_residual = np.array(alpha3_coupling_residual_list)
    alpha4_coupling_residual = np.array(alpha4_coupling_residual_list)

    # =========================
    # Print summary
    # =========================

    print("\n=== Near-segment proportionality check ===")
    print(f"alpha1 - K*u_ax: max_abs = {np.max(np.abs(alpha1_residual)):.3e}, mean_abs = {np.mean(np.abs(alpha1_residual)):.3e}")
    print(f"alpha2 - K*u_ay: max_abs = {np.max(np.abs(alpha2_residual)):.3e}, mean_abs = {np.mean(np.abs(alpha2_residual)):.3e}")

    print("\n=== Distal coupling decomposition check ===")
    print(f"alpha3 - (prox+dist): max_abs = {np.max(np.abs(alpha3_coupling_residual)):.3e}, mean_abs = {np.mean(np.abs(alpha3_coupling_residual)):.3e}")
    print(f"alpha4 - (prox+dist): max_abs = {np.max(np.abs(alpha4_coupling_residual)):.3e}, mean_abs = {np.mean(np.abs(alpha4_coupling_residual)):.3e}")

    # show why distal is not simple K*u_cx or K*u_cy
    alpha3_minus_K_ucx = alpha3 - K * u_cx
    alpha4_minus_K_ucy = alpha4 - K * u_cy

    print("\n=== Distal NOT equal to simple local CC scaling ===")
    print(f"alpha3 - K*u_cx: max_abs = {np.max(np.abs(alpha3_minus_K_ucx)):.3e}, mean_abs = {np.mean(np.abs(alpha3_minus_K_ucx)):.3e}")
    print(f"alpha4 - K*u_cy: max_abs = {np.max(np.abs(alpha4_minus_K_ucy)):.3e}, mean_abs = {np.mean(np.abs(alpha4_minus_K_ucy)):.3e}")

    # =========================
    # Plot 1: alpha1 vs u_ax
    # =========================
    plt.figure(figsize=(6, 5))
    plt.scatter(u_ax, alpha1, s=8, alpha=0.5, label="Samples")
    xline = np.linspace(np.min(u_ax), np.max(u_ax), 200)
    plt.plot(xline, K * xline, linewidth=2, label="alpha1 = K * u_ax")
    plt.xlabel("u_ax")
    plt.ylabel("alpha1")
    plt.title("Near Segment: alpha1 vs u_ax")
    plt.legend()
    plt.grid(True)

    # =========================
    # Plot 2: alpha2 vs u_ay
    # =========================
    plt.figure(figsize=(6, 5))
    plt.scatter(u_ay, alpha2, s=8, alpha=0.5, label="Samples")
    xline = np.linspace(np.min(u_ay), np.max(u_ay), 200)
    plt.plot(xline, K * xline, linewidth=2, label="alpha2 = K * u_ay")
    plt.xlabel("u_ay")
    plt.ylabel("alpha2")
    plt.title("Near Segment: alpha2 vs u_ay")
    plt.legend()
    plt.grid(True)

    # =========================
    # Plot 3: alpha3 decomposition
    # =========================
    idx = np.arange(len(alpha3))
    plt.figure(figsize=(10, 5))
    plt.plot(idx, alpha3, linewidth=1.2, label="alpha3 actual")
    plt.plot(idx, alpha3_prox, linewidth=1.0, label="alpha3 proximal part")
    plt.plot(idx, alpha3_dist, linewidth=1.0, label="alpha3 distal part")
    plt.plot(idx, alpha3_sum, linewidth=1.0, linestyle="--", label="alpha3 sum")
    plt.xlabel("Sample index")
    plt.ylabel("Value")
    plt.title("Distal Motor alpha3: Proximal-Distal Coupling")
    plt.legend()
    plt.grid(True)

    # =========================
    # Plot 4: alpha4 decomposition
    # =========================
    idx = np.arange(len(alpha4))
    plt.figure(figsize=(10, 5))
    plt.plot(idx, alpha4, linewidth=1.2, label="alpha4 actual")
    plt.plot(idx, alpha4_prox, linewidth=1.0, label="alpha4 proximal part")
    plt.plot(idx, alpha4_dist, linewidth=1.0, label="alpha4 distal part")
    plt.plot(idx, alpha4_sum, linewidth=1.0, linestyle="--", label="alpha4 sum")
    plt.xlabel("Sample index")
    plt.ylabel("Value")
    plt.title("Distal Motor alpha4: Proximal-Distal Coupling")
    plt.legend()
    plt.grid(True)

    # =========================
    # Plot 5: show distal is NOT simple K*u_cx
    # =========================
    plt.figure(figsize=(6, 5))
    plt.scatter(K * u_cx, alpha3, s=8, alpha=0.5)
    plt.xlabel("K * u_cx")
    plt.ylabel("alpha3")
    plt.title("Distal: alpha3 is NOT simply K * u_cx")
    plt.grid(True)

    # =========================
    # Plot 6: show distal is NOT simple K*u_cy
    # =========================
    plt.figure(figsize=(6, 5))
    plt.scatter(K * u_cy, alpha4, s=8, alpha=0.5)
    plt.xlabel("K * u_cy")
    plt.ylabel("alpha4")
    plt.title("Distal: alpha4 is NOT simply K * u_cy")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
