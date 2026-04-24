# packages/robot_msgs/tests/test_serialization.py

from robot_msgs import JointTarget, MotorTarget, PmacCommand
from robot_msgs.serialization import dumps_msg, loads_msg


def test_joint_target_serialization():
    msg = JointTarget(
        phi_a=0.1,
        theta_a=0.2,
        phi_c=-0.1,
        theta_c=0.3,
    )

    data = dumps_msg(msg)
    restored = loads_msg(data, JointTarget)

    assert restored.msg_type == "joint_target"
    assert restored.phi_a == 0.1
    assert restored.theta_c == 0.3


def test_motor_target_count():
    msg = MotorTarget(motor_angles_rad=[0.1, 0.2, 0.3, 0.4])
    assert len(msg.motor_angles_rad) == 4


def test_pmac_command_axis_count():
    msg = PmacCommand(
        target_pulses=[0, 1, 2, 3, 4],
        move_time_ms=500,
        accel=100,
        scurve=50,
    )

    assert len(msg.target_pulses) == 5
