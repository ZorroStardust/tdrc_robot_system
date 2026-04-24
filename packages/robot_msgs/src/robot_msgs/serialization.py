# packages/robot_msgs/src/robot_msgs/serialization.py

from __future__ import annotations

import json
from typing import Type, TypeVar
from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


def dumps_msg(msg: BaseModel) -> bytes:
    return msg.model_dump_json().encode("utf-8")


def loads_msg(data: bytes, msg_cls: Type[T]) -> T:
    return msg_cls.model_validate_json(data.decode("utf-8"))


def dumps_topic_msg(topic: str, msg: BaseModel) -> tuple[bytes, bytes]:
    return topic.encode("utf-8"), dumps_msg(msg)


def loads_topic_msg(topic_data: bytes, msg_data: bytes, msg_cls: Type[T]) -> tuple[str, T]:
    topic = topic_data.decode("utf-8")
    msg = loads_msg(msg_data, msg_cls)
    return topic, msg
