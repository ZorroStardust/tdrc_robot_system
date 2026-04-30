from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent

for relative_path in (
    "packages/pmac_sdk/src",
    "packages/robot_msgs/src",
    "packages/motion_coordinator/src",
    "packages/pmac_bridge_node/src",
    "packages/tdrc_model_node/src/python_src",
):
    package_path = REPO_ROOT / relative_path
    if package_path.exists():
        package_path_str = str(package_path)
        if package_path_str not in sys.path:
            sys.path.insert(0, package_path_str)