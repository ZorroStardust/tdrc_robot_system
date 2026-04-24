# -*- coding: utf-8 -*-
"""
test_numeric.py

Pure numerical tests for model.py

Covers:
1) Symmetry checks
2) Forward consistency checks
3) Inverse closure checks
4) Degenerate / singular cases
"""

from __future__ import annotations

import math
import random
import unittest

from model import (
    JointSpace,
    MotorAngles,
    TDRCJointMotorModel,
    wrap_to_pi,
    angle_diff,
)


# =========================
# Config
# =========================

R_HOLE = 0.003       # [m] example hole radius
D_SPOOL = 0.012      # [m] example spool diameter

THETA_A_MAX = math.pi         # proximal max bending
THETA_C_MAX = math.pi / 2.0   # distal max bending

N_RANDOM_SAMPLES = 1000
ANGLE_ABS_TOL = 1e-9
VALUE_ABS_TOL = 1e-10


# =========================
# Helpers
# =========================

def assert_float_close(testcase: unittest.TestCase, a: float, b: float, tol: float = VALUE_ABS_TOL, msg: str = ""):
    testcase.assertTrue(
        math.isclose(a, b, abs_tol=tol, rel_tol=0.0),
        msg or f"Not close: {a} vs {b}, tol={tol}"
    )


def assert_angle_close(testcase: unittest.TestCase, a: float, b: float, tol: float = ANGLE_ABS_TOL, msg: str = ""):
    err = abs(angle_diff(a, b))
    testcase.assertTrue(
        err <= tol,
        msg or f"Angle not close: {a} vs {b}, wrapped err={err}, tol={tol}"
    )


def random_joint(
    theta_a_max: float = THETA_A_MAX,
    theta_c_max: float = THETA_C_MAX,
) -> JointSpace:
    return JointSpace(
        phi_a=random.uniform(-math.pi, math.pi),
        theta_a=random.uniform(0.0, theta_a_max),
        phi_c=random.uniform(-math.pi, math.pi),
        theta_c=random.uniform(0.0, theta_c_max),
    )


def non_singular_random_joint(
    theta_a_max: float = THETA_A_MAX,
    theta_c_max: float = THETA_C_MAX,
    theta_min: float = 1e-4,
) -> JointSpace:
    return JointSpace(
        phi_a=random.uniform(-math.pi, math.pi),
        theta_a=random.uniform(theta_min, theta_a_max),
        phi_c=random.uniform(-math.pi, math.pi),
        theta_c=random.uniform(theta_min, theta_c_max),
    )


# =========================
# Test cases
# =========================

class TestTDRCJointMotorModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            zero_eps=1e-10,
        )

    # -------------------------
    # Basic model parameter tests
    # -------------------------

    def test_K_definition(self):
        expected_K = 2.0 * R_HOLE / D_SPOOL
        assert_float_close(self, self.model.K, expected_K)

    def test_gamma_a_definition(self):
        expected = {
            1: 0.0,
            2: math.pi / 2.0,
            3: math.pi,
            4: 3.0 * math.pi / 2.0,
            5: math.pi / 4.0,
            6: 3.0 * math.pi / 4.0,
            7: 5.0 * math.pi / 4.0,
            8: 7.0 * math.pi / 4.0,
        }
        for idx, val in expected.items():
            assert_angle_close(self, self.model.gamma_a(idx), val)

    def test_gamma_c_definition(self):
        expected = {
            5: math.pi / 4.0,
            6: 3.0 * math.pi / 4.0,
            7: 5.0 * math.pi / 4.0,
            8: 7.0 * math.pi / 4.0,
        }
        for idx, val in expected.items():
            assert_angle_close(self, self.model.gamma_c(idx), val)

    # -------------------------
    # Symmetry tests
    # -------------------------

    def test_proximal_tendon_symmetry(self):
        for _ in range(N_RANDOM_SAMPLES):
            joint = random_joint()
            tendon = self.model.joint_to_tendon_lengths(joint)

            assert_float_close(self, tendon.dl3, -tendon.dl1, tol=1e-12, msg="dl3 != -dl1")
            assert_float_close(self, tendon.dl4, -tendon.dl2, tol=1e-12, msg="dl4 != -dl2")

    def test_distal_tendon_symmetry(self):
        for _ in range(N_RANDOM_SAMPLES):
            joint = random_joint()
            tendon = self.model.joint_to_tendon_lengths(joint)

            assert_float_close(self, tendon.dl7, -tendon.dl5, tol=1e-12, msg="dl7 != -dl5")
            assert_float_close(self, tendon.dl8, -tendon.dl6, tol=1e-12, msg="dl8 != -dl6")

    # -------------------------
    # CC component tests
    # -------------------------

    def test_cc_components_definition(self):
        joint = JointSpace(
            phi_a=0.3,
            theta_a=0.8,
            phi_c=-0.4,
            theta_c=0.5,
        )
        cc = self.model.joint_to_cc_components(joint)

        assert_float_close(self, cc.u_ax, -joint.theta_a * math.cos(joint.phi_a))
        assert_float_close(self, cc.u_ay, -joint.theta_a * math.sin(joint.phi_a))
        assert_float_close(self, cc.u_cx, -joint.theta_c * math.cos(joint.phi_c))
        assert_float_close(self, cc.u_cy, -joint.theta_c * math.sin(joint.phi_c))

    def test_zero_joint_gives_zero_cc(self):
        joint = JointSpace(phi_a=0.0, theta_a=0.0, phi_c=0.0, theta_c=0.0)
        cc = self.model.joint_to_cc_components(joint)

        assert_float_close(self, cc.u_ax, 0.0)
        assert_float_close(self, cc.u_ay, 0.0)
        assert_float_close(self, cc.u_cx, 0.0)
        assert_float_close(self, cc.u_cy, 0.0)

    # -------------------------
    # Forward consistency tests
    # -------------------------

    def test_motor_from_tendon_matches_direct_motor(self):
        for _ in range(N_RANDOM_SAMPLES):
            joint = random_joint()

            tendon = self.model.joint_to_tendon_lengths(joint)
            motor_from_tendon = self.model.tendon_lengths_to_motor_angles(tendon)
            motor_direct = self.model.joint_to_motor_angles(joint)

            assert_float_close(self, motor_from_tendon.alpha1, motor_direct.alpha1)
            assert_float_close(self, motor_from_tendon.alpha2, motor_direct.alpha2)
            assert_float_close(self, motor_from_tendon.alpha3, motor_direct.alpha3)
            assert_float_close(self, motor_from_tendon.alpha4, motor_direct.alpha4)

    def test_motor_index_mapping_swap_1_2(self):
        # Example: physical motor 1/2 are swapped compared with ideal numbering.
        mapped_model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            zero_eps=1e-10,
            motor_index_map={1: 2, 2: 1, 3: 3, 4: 4},
        )

        joint = JointSpace(
            phi_a=0.3,
            theta_a=0.8,
            phi_c=-0.4,
            theta_c=0.5,
        )

        motor_ideal = self.model.joint_to_motor_angles(joint)
        motor_real = mapped_model.joint_to_motor_angles(joint)

        # ideal->real mapping: 1->2, 2->1, 3->3, 4->4
        assert_float_close(self, motor_real.alpha1, motor_ideal.alpha2)
        assert_float_close(self, motor_real.alpha2, motor_ideal.alpha1)
        assert_float_close(self, motor_real.alpha3, motor_ideal.alpha3)
        assert_float_close(self, motor_real.alpha4, motor_ideal.alpha4)

        # inverse should still recover same joint when given real-indexed motor data
        recovered = mapped_model.motor_angles_to_joint(motor_real)
        assert_float_close(self, recovered.theta_a, joint.theta_a, tol=1e-9)
        assert_float_close(self, recovered.theta_c, joint.theta_c, tol=1e-9)
        assert_angle_close(self, recovered.phi_a, joint.phi_a, tol=1e-9)
        assert_angle_close(self, recovered.phi_c, joint.phi_c, tol=1e-9)

    def test_invalid_motor_index_mapping_raises(self):
        with self.assertRaises(ValueError):
            TDRCJointMotorModel(
                hole_radius=R_HOLE,
                spool_diameter=D_SPOOL,
                motor_index_map={1: 2, 2: 1, 3: 3},
            )

        with self.assertRaises(ValueError):
            TDRCJointMotorModel(
                hole_radius=R_HOLE,
                spool_diameter=D_SPOOL,
                motor_index_map={1: 2, 2: 1, 3: 3, 4: 3},
            )

    def test_motor_direction_mapping_sign_flip(self):
        mapped_model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            zero_eps=1e-10,
            motor_direction_map={1: -1, 2: 1, 3: -1, 4: 1},
        )

        joint = JointSpace(
            phi_a=0.3,
            theta_a=0.8,
            phi_c=-0.4,
            theta_c=0.5,
        )

        motor_ideal = self.model.joint_to_motor_angles(joint)
        motor_real = mapped_model.joint_to_motor_angles(joint)

        assert_float_close(self, motor_real.alpha1, -motor_ideal.alpha1)
        assert_float_close(self, motor_real.alpha2, motor_ideal.alpha2)
        assert_float_close(self, motor_real.alpha3, -motor_ideal.alpha3)
        assert_float_close(self, motor_real.alpha4, motor_ideal.alpha4)

        recovered = mapped_model.motor_angles_to_joint(motor_real)
        assert_float_close(self, recovered.theta_a, joint.theta_a, tol=1e-9)
        assert_float_close(self, recovered.theta_c, joint.theta_c, tol=1e-9)
        assert_angle_close(self, recovered.phi_a, joint.phi_a, tol=1e-9)
        assert_angle_close(self, recovered.phi_c, joint.phi_c, tol=1e-9)

    def test_invalid_motor_direction_mapping_raises(self):
        with self.assertRaises(ValueError):
            TDRCJointMotorModel(
                hole_radius=R_HOLE,
                spool_diameter=D_SPOOL,
                motor_direction_map={1: -1, 2: 1, 3: -1},
            )

        with self.assertRaises(ValueError):
            TDRCJointMotorModel(
                hole_radius=R_HOLE,
                spool_diameter=D_SPOOL,
                motor_direction_map={1: -1, 2: 1, 3: 0, 4: 1},
            )

    def test_zero_joint_gives_zero_tendon_and_zero_motor(self):
        joint = JointSpace(phi_a=0.0, theta_a=0.0, phi_c=0.0, theta_c=0.0)

        tendon = self.model.joint_to_tendon_lengths(joint)
        motor = self.model.joint_to_motor_angles(joint)

        for val in tendon.as_tuple():
            assert_float_close(self, val, 0.0)

        for val in motor.as_tuple():
            assert_float_close(self, val, 0.0)

    def test_known_example_matches_expected_pattern(self):
        joint = JointSpace(
            phi_a=0.3,
            theta_a=0.8,
            phi_c=-0.4,
            theta_c=0.5,
        )

        tendon = self.model.joint_to_tendon_lengths(joint)
        motor = self.model.joint_to_motor_angles(joint)

        # proximal symmetry
        assert_float_close(self, tendon.dl3, -tendon.dl1)
        assert_float_close(self, tendon.dl4, -tendon.dl2)

        # distal symmetry
        assert_float_close(self, tendon.dl7, -tendon.dl5)
        assert_float_close(self, tendon.dl8, -tendon.dl6)

        # motor sign/definition consistency
        scale = -2.0 / D_SPOOL
        assert_float_close(self, motor.alpha1, scale * tendon.dl1)
        assert_float_close(self, motor.alpha2, scale * tendon.dl2)
        assert_float_close(self, motor.alpha3, scale * tendon.dl5)
        assert_float_close(self, motor.alpha4, scale * tendon.dl6)

    # -------------------------
    # Inverse closure tests
    # -------------------------

    def test_motor_to_joint_closure_non_singular(self):
        for _ in range(N_RANDOM_SAMPLES):
            joint = non_singular_random_joint(theta_min=1e-4)
            motor = self.model.joint_to_motor_angles(joint)
            recovered = self.model.motor_angles_to_joint(motor)

            self.assertFalse(recovered.singular_a, "Unexpected singular_a in non-singular test")
            self.assertFalse(recovered.singular_c, "Unexpected singular_c in non-singular test")

            assert_float_close(self, recovered.theta_a, joint.theta_a, tol=1e-9)
            assert_float_close(self, recovered.theta_c, joint.theta_c, tol=1e-9)
            assert_angle_close(self, recovered.phi_a, joint.phi_a, tol=1e-9)
            assert_angle_close(self, recovered.phi_c, joint.phi_c, tol=1e-9)

    def test_joint_to_motor_to_joint_to_motor_roundtrip(self):
        for _ in range(N_RANDOM_SAMPLES):
            joint = random_joint()
            motor_1 = self.model.joint_to_motor_angles(joint)
            recovered = self.model.motor_angles_to_joint(motor_1)

            joint_2 = JointSpace(
                phi_a=recovered.phi_a,
                theta_a=recovered.theta_a,
                phi_c=recovered.phi_c,
                theta_c=recovered.theta_c,
            )
            motor_2 = self.model.joint_to_motor_angles(joint_2)

            assert_float_close(self, motor_1.alpha1, motor_2.alpha1, tol=1e-9)
            assert_float_close(self, motor_1.alpha2, motor_2.alpha2, tol=1e-9)
            assert_float_close(self, motor_1.alpha3, motor_2.alpha3, tol=1e-9)
            assert_float_close(self, motor_1.alpha4, motor_2.alpha4, tol=1e-9)

    # -------------------------
    # Singular / degenerate cases
    # -------------------------

    def test_singular_a_only(self):
        joint = JointSpace(
            phi_a=1.2,     # physically irrelevant when theta_a = 0
            theta_a=0.0,
            phi_c=-0.7,
            theta_c=0.6,
        )

        motor = self.model.joint_to_motor_angles(joint)
        recovered = self.model.motor_angles_to_joint(motor)

        self.assertTrue(recovered.singular_a)
        self.assertFalse(recovered.singular_c)

        assert_float_close(self, recovered.theta_a, 0.0, tol=1e-12)
        assert_float_close(self, recovered.theta_c, joint.theta_c, tol=1e-9)
        assert_angle_close(self, recovered.phi_c, joint.phi_c, tol=1e-9)

    def test_singular_c_only(self):
        joint = JointSpace(
            phi_a=0.8,
            theta_a=0.9,
            phi_c=-2.0,    # physically irrelevant when theta_c = 0
            theta_c=0.0,
        )

        motor = self.model.joint_to_motor_angles(joint)
        recovered = self.model.motor_angles_to_joint(motor)

        self.assertFalse(recovered.singular_a)
        self.assertTrue(recovered.singular_c)

        assert_float_close(self, recovered.theta_a, joint.theta_a, tol=1e-9)
        assert_angle_close(self, recovered.phi_a, joint.phi_a, tol=1e-9)
        assert_float_close(self, recovered.theta_c, 0.0, tol=1e-12)

    def test_both_segments_singular(self):
        joint = JointSpace(
            phi_a=1.0,
            theta_a=0.0,
            phi_c=-1.0,
            theta_c=0.0,
        )

        motor = self.model.joint_to_motor_angles(joint)
        recovered = self.model.motor_angles_to_joint(motor)

        self.assertTrue(recovered.singular_a)
        self.assertTrue(recovered.singular_c)

        assert_float_close(self, recovered.theta_a, 0.0, tol=1e-12)
        assert_float_close(self, recovered.theta_c, 0.0, tol=1e-12)

        # By current convention in model.py:
        assert_float_close(self, recovered.phi_a, 0.0, tol=1e-12)
        assert_float_close(self, recovered.phi_c, 0.0, tol=1e-12)

    def test_near_singular_threshold(self):
        model = TDRCJointMotorModel(
            hole_radius=R_HOLE,
            spool_diameter=D_SPOOL,
            cc_sign=-1.0,
            zero_eps=1e-8,
        )

        joint = JointSpace(
            phi_a=0.5,
            theta_a=1e-10,
            phi_c=-0.3,
            theta_c=1e-10,
        )

        motor = model.joint_to_motor_angles(joint)
        recovered = model.motor_angles_to_joint(motor)

        self.assertTrue(recovered.singular_a)
        self.assertTrue(recovered.singular_c)

    # -------------------------
    # Utility function tests
    # -------------------------

    def test_wrap_to_pi_range(self):
        samples = [
            -10.0 * math.pi,
            -3.0 * math.pi,
            -2.1,
            -0.1,
            0.0,
            0.1,
            2.1,
            3.0 * math.pi,
            10.0 * math.pi,
        ]
        for a in samples:
            w = wrap_to_pi(a)
            self.assertTrue(-math.pi <= w < math.pi, f"wrap_to_pi out of range: {w}")

    def test_angle_diff_basic(self):
        a = math.pi - 0.01
        b = -math.pi + 0.01
        d = angle_diff(a, b)
        self.assertTrue(abs(d + 0.02) < 1e-9 or abs(d - (2*math.pi - 0.02)) > 1e-3)

    # -------------------------
    # Explicit inverse formula spot-check
    # -------------------------

    def test_inverse_from_manual_motor_example(self):
        joint = JointSpace(
            phi_a=0.3,
            theta_a=0.8,
            phi_c=-0.4,
            theta_c=0.5,
        )

        motor = MotorAngles(
            alpha1=-0.38213459565024244,
            alpha2=-0.11820808266453582,
            alpha3=-0.4477777317986668,
            alpha4=0.41828643339479576,
        )

        recovered = self.model.motor_angles_to_joint(motor)

        assert_float_close(self, recovered.theta_a, joint.theta_a, tol=1e-9)
        assert_float_close(self, recovered.theta_c, joint.theta_c, tol=1e-9)
        assert_angle_close(self, recovered.phi_a, joint.phi_a, tol=1e-9)
        assert_angle_close(self, recovered.phi_c, joint.phi_c, tol=1e-9)


if __name__ == "__main__":
    random.seed(42)
    unittest.main(verbosity=2)
