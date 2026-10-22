#!/usr/bin/env python3
"""Bring a problem up to date with the measurements and plan it again.

    ros2 run lab_replica replan_problem.py <problem> [--no-settle] [--render]

<problem> is a name in problems/ (e.g. all_on_table) or a directory.
Run it after correcting any measurement (config/cell.yaml, config/items.yaml)
or where an item stands (the problem's layout.yaml). It

  1. rebuilds the Gazebo models from cell.yaml and items.yaml;
  2. puts every item where layout.yaml says (x, y and yaw_deg; the height
     follows from its size; items in the basket move with the basket);
  3. settles the arrangement in headless Gazebo and warns about items that
     move or tip over there (overlapping items, a wrong resting pose);
  4. plans every motion again, collision-checked;
  5. only if that succeeds, rewrites world.sdf, plan.json, layout.png,
     solution.py and README.md of the problem (about 30 s in all).

If no collision-free plan exists it says which item and obstacle are in the
way and changes nothing. layout.yaml itself is never rewritten.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config  # noqa: E402
from lab_replica.problems.replan import ReplanError, replan  # noqa: E402


def main():
    sys.stdout.reconfigure(line_buffering=True)   # keep messages in order with stderr
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("problem")
    ap.add_argument("--no-settle", action="store_true",
                    help="skip the Gazebo settling check and use the measured poses as they are")
    ap.add_argument("--no-models", action="store_true", help="do not rebuild the Gazebo models")
    ap.add_argument("--render", action="store_true", help="also render render.png (the camera view; slow)")
    args = ap.parse_args()
    d = config.problem_dir(args.problem)
    try:
        replan(d, settle=not args.no_settle, rebuild_models=not args.no_models, render=args.render)
    except ReplanError as e:
        print(f"FAILED: {e}\nNothing in {d} was changed. Check the measurements, or place the items so "
              f"the gripper has room around them, edit layout.yaml accordingly and run this again.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
