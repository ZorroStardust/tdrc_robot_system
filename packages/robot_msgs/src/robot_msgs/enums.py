# packages/robot_msgs/src/robot_msgs/enums.py

from enum import Enum


class ControlMode(str, Enum):
    IDLE = "idle"
    BOOTING = "booting"
    HOMING = "homing"
    ZEROING = "zeroing"
    POSITION = "position"
    TRAJECTORY = "trajectory"
    STREAMING = "streaming"
    FAULT = "fault"
    ESTOP = "estop"


class CommandType(str, Enum):
    SET_MODE = "set_mode"
    BOOT = "boot"
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    ZERO_CURRENT = "zero_current"
    RESET_FAULT = "reset_fault"
    STOP = "stop"
    ESTOP = "estop"


class MotionType(str, Enum):
    POSITION = "position"
    TRAJECTORY_POINT = "trajectory_point"
    STREAMING = "streaming"


class ErrorLevel(str, Enum):
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    FATAL = "fatal"
