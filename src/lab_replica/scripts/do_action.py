#!/usr/bin/env python3
"""Ask the skill server (skill_server.py) to execute PDDL actions, in order.

    ros2 run lab_replica do_action.py "(pick-up green-mug)" "(put-in-basket green-mug)"
    ros2 run lab_replica do_action.py "(put-on-table red-block)" --at 0.55 -0.10

Prints the phases as they happen; exits 0 when every action succeeded (it
stops at the first failure).
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("actions", nargs="+")
    ap.add_argument("--at", nargs=2, type=float, metavar=("X", "Y"), help="put-on-table: where (world x y)")
    args = ap.parse_args()
    from lab_skills_msgs.action import ExecuteAction
    rclpy.init()
    node = Node("do_action")
    cli = ActionClient(node, ExecuteAction, "/lab_skills/execute")
    if not cli.wait_for_server(timeout_sec=30.0):
        print("no /lab_skills/execute: start skill_server.py", file=sys.stderr)
        return 2
    for a in args.actions:
        goal = ExecuteAction.Goal(action=a, target_xy=list(args.at) if args.at else [])
        print(f"== {a}")
        fut = cli.send_goal_async(goal, feedback_callback=lambda f: print(f"   {f.feedback.phase}"))
        rclpy.spin_until_future_complete(node, fut)
        gh = fut.result()
        if not gh.accepted:
            print("rejected", file=sys.stderr)
            return 1
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf)
        r = rf.result().result
        print(f"{'OK' if r.success else 'FAILED'}: {r.message}" + (f" (holding {r.held})" if r.held else ""))
        if not r.success:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
