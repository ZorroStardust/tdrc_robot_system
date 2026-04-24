from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "packages/motion_coordinator/src"))
sys.path.insert(0, str(REPO_ROOT / "packages/robot_msgs/src"))
