# packages/robot_msgs/src/robot_msgs/base.py

from __future__ import annotations

import time
from typing import Any, Dict
from pydantic import BaseModel, Field


class MsgHeader(BaseModel):
    stamp: float = Field(default_factory=time.time)    # 消息生成时间
    seq: int = 0                                       # 消息序号
    source: str = "unknown"                            # 消息来源节点
    frame_id: str = ""                                 # 坐标系或参考系


class BaseMsg(BaseModel):
    msg_type: str
    header: MsgHeader = Field(default_factory=MsgHeader)

    def to_dict(self) -> Dict[str, Any]:
        return self.model_dump()

    def to_json(self) -> str:
        return self.model_dump_json()
