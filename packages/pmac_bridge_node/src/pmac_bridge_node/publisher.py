import zmq
import time
from .bridge import PMACBridge

def start_publisher(stop_event=None):
    bridge = PMACBridge(controller=None)  # 初始化 PMAC 控制器
    bridge.start_publisher(stop_event=stop_event)
