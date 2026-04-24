# packages/robot_msgs/src/robot_msgs/command.py

from typing import Any, Dict
from pydantic import Field
from .base import BaseMsg
from .enums import CommandType, ControlMode


class SystemCommand(BaseMsg):
    msg_type: str = "system_command"

    command: CommandType
    target_mode: ControlMode | None = None
    params: Dict[str, Any] = Field(default_factory=dict)


class CommandReply(BaseMsg):
    msg_type: str = "command_reply"

    ok: bool
    command: CommandType | None = None
    message: str = ""
    data: Dict[str, Any] = Field(default_factory=dict)
