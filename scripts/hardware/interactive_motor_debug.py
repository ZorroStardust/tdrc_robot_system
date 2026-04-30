import argparse
import shlex
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "packages" / "pmac_sdk" / "src"))

from pmac_sdk.controller.robot_api import PMACRobotController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.200")
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--unit", type=int, default=1)
    args = parser.parse_args()

    robot = PMACRobotController(host=args.host, port=args.port, unit_id=args.unit)
    robot.connect()

    print("Connected.")
    print("Commands:")
    print("  status")
    print("  startup")
    print("  reset")
    print("  csp")
    print("  enable")
    print("  hold")
    print("  stop")
    print("  disable")
    print("  pos")
    print("  move p1 p2 p3 p4 p5 [time accel scurve]")
    print("  rel d1 d2 d3 d4 d5 [time accel scurve]")
    print("  quit")

    try:
        while True:
            line = input("pmac> ").strip()
            if not line:
                continue

            parts = shlex.split(line)
            cmd = parts[0].lower()

            try:
                if cmd in ("quit", "exit", "q"):
                    break

                elif cmd == "status":
                    robot.print_status()

                elif cmd == "startup":
                    status = robot.startup_sequence()
                    print(status)

                elif cmd == "reset":
                    print(robot.reset_fault())

                elif cmd == "csp":
                    print(robot.set_csp_mode())

                elif cmd == "enable":
                    print(robot.enable_operation())

                elif cmd == "hold":
                    print(robot.hold_current_position())

                elif cmd == "stop":
                    print(robot.stop())

                elif cmd == "disable":
                    print(robot.disable_operation())

                elif cmd == "pos":
                    print(robot.read_actual_positions())

                elif cmd == "move":
                    if len(parts) < 6:
                        print("Usage: move p1 p2 p3 p4 p5 [time accel scurve]")
                        continue

                    pos = [int(x) for x in parts[1:6]]
                    move_time = int(parts[6]) if len(parts) > 6 else 500
                    accel = int(parts[7]) if len(parts) > 7 else 100
                    scurve = int(parts[8]) if len(parts) > 8 else 50

                    print(robot.move_joints(pos, move_time, accel, scurve))

                elif cmd == "rel":
                    if len(parts) < 6:
                        print("Usage: rel d1 d2 d3 d4 d5 [time accel scurve]")
                        continue

                    delta = [int(x) for x in parts[1:6]]
                    move_time = int(parts[6]) if len(parts) > 6 else 500
                    accel = int(parts[7]) if len(parts) > 7 else 100
                    scurve = int(parts[8]) if len(parts) > 8 else 50

                    print(robot.move_relative(delta, move_time, accel, scurve))

                else:
                    print(f"Unknown command: {cmd}")

            except Exception as exc:
                print(f"ERROR: {exc}")

    finally:
        robot.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
