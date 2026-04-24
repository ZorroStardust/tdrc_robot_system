# packages/robot_msgs/src/robot_msgs/joint.py

from pydantic import Field
from .base import BaseMsg
from .enums import MotionType


class JointTarget(BaseMsg):
    msg_type: str = "joint_target"

    phi_a: float = 0.0
    theta_a: float = 0.0
    phi_c: float = 0.0
    theta_c: float = 0.0

    unit: str = "rad"
    motion_type: MotionType = MotionType.POSITION

    ttl_ms: int = Field(default=200, description="Message time-to-live in milliseconds")
