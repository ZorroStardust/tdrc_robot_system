import zmq
from robot_msgs import MotorTarget
from .bridge import PMACBridge

def start_subscriber(stop_event=None):
    bridge = PMACBridge(controller=None)  # 初始化 PMAC 控制器
    bridge.start_subscriber(stop_event=stop_event)
