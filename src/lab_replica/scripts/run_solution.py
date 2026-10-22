#!/usr/bin/env python3
"""Run a problem's hard-coded solution (its solution.py).

    ros2 run lab_replica run_solution.py <problem> [--speed-scale S] [--step] ...

<problem> is a name in problems/ (e.g. all_on_table) or a directory.
It drives whatever is running, with the same commands: the simulation
(ros2 launch lab_replica sim.launch.py problem:=<problem>) or the real robot
(ros2 launch lab_replica real.launch.py). If the arm is not at the start
pose, it first offers a slow joint move there. Ctrl-C stops the arm.
"""

import argparse
import os
import runpy
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config  # noqa: E402
from lab_replica.execution import runner  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("problem")
    runner.add_args(ap)
    args = ap.parse_args()
    solution = runpy.run_path(os.path.join(config.problem_dir(args.problem), "solution.py"), run_name="solution")
    title = (solution.get("__doc__") or "").splitlines()[0]
    sys.exit(runner.run(solution["SETUP"], solution["ACTIONS"], args, title=title))


if __name__ == "__main__":
    main()
