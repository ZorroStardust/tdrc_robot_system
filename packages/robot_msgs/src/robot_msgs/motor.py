# packages/robot_msgs/src/robot_msgs/motor.py

from typing import List
from pydantic import Field, field_validator
from .base import BaseMsg
from .enums import MotionType


class MotorTarget(BaseMsg):
    msg_type: str = "motor_target"

    motor_angles_rad: List[float] = Field(default_factory=list)
    motion_type: MotionType = MotionType.POSITION

    move_time_ms: int = 500
    ttl_ms: int = 200

    @field_validator("motor_angles_rad")
    @classmethod
    def validate_motor_count(cls, value: List[float]) -> List[float]:
        if len(value) not in (4, 5):
            raise ValueError("motor_angles_rad must contain 4 or 5 values")
        return value
