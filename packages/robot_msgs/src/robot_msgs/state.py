# packages/robot_msgs/src/robot_msgs/state.py

from typing import Dict, Any, List, Optional
from pydantic import Field
from .base import BaseMsg
from .enums import ControlMode, ErrorLevel


class ErrorReport(BaseMsg):
    msg_type: str = "error_report"

    level: ErrorLevel = ErrorLevel.ERROR
    code: str = ""
    message: str = ""
    detail: Dict[str, Any] = Field(default_factory=dict)


class RobotState(BaseMsg):
    msg_type: str = "robot_state"

    mode: ControlMode = ControlMode.IDLE
    connected: bool = False
    enabled: bool = False
    fault: bool = False

    motor_angles_rad: Optional[List[float]] = None
    motor_pulses: Optional[List[int]] = None

    joint_estimate: Optional[Dict[str, float]] = None

    last_error: Optional[str] = None
