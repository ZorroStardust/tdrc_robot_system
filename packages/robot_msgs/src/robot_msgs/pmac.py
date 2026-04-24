# packages/robot_msgs/src/robot_msgs/pmac.py

from typing import List, Optional
from pydantic import Field, field_validator
from .base import BaseMsg


class PmacCommand(BaseMsg):
    msg_type: str = "pmac_command"

    target_pulses: List[int] = Field(default_factory=list)
    move_time_ms: int = 500
    accel: int = 100
    scurve: int = 50

    @field_validator("target_pulses")
    @classmethod
    def validate_axis_count(cls, value: List[int]) -> List[int]:
        if len(value) != 5:
            raise ValueError("target_pulses must contain exactly 5 axis values")
        return value


class PmacState(BaseMsg):
    msg_type: str = "pmac_state"

    connected: bool = False
    enabled: bool = False
    fault: bool = False

    current_pulses: List[int] = Field(default_factory=list)
    target_pulses: Optional[List[int]] = None

    message: str = ""
