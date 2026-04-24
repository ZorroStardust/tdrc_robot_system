from pathlib import Path
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "packages/pmac_bridge_node/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/robot_msgs/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/pmac_sdk/src"))

from pmac_bridge_node.subscriber import start_subscriber
from pmac_bridge_node.publisher import start_publisher

def main():
    # 并发启动 ZMQ 消息接收与发布，并支持 Ctrl+C 优雅退出
    stop_event = threading.Event()

    subscriber_thread = threading.Thread(
        target=start_subscriber,
        kwargs={"stop_event": stop_event},
        name="pmac-subscriber",
        daemon=True,
    )
    publisher_thread = threading.Thread(
        target=start_publisher,
        kwargs={"stop_event": stop_event},
        name="pmac-publisher",
        daemon=True,
    )

    subscriber_thread.start()
    publisher_thread.start()
    print("[PMACBridge] Running. Press Ctrl+C to stop.")

    try:
        while subscriber_thread.is_alive() and publisher_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[PMACBridge] Ctrl+C received, stopping...")
    finally:
        stop_event.set()
        subscriber_thread.join(timeout=2)
        publisher_thread.join(timeout=2)
        print("[PMACBridge] Exited")

if __name__ == "__main__":
    main()
