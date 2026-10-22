#!/usr/bin/env python3
"""Generate random, physically valid problems for the lab replica.

    ros2 run lab_replica generate_problem.py [--count N] [--seed S] [--name NAME]
                        [--basket-items K] [--p-holding P]
                        [--min-actions 2] [--max-actions 10]
                        [--randomize-basket] [--no-render]

Each problem goes to problems/<name>/ (default problems/problem_<seed>/),
with the same files as problems/all_on_table (see its README.md),
including a hard-coded solution.py.

What "valid" means here:
  * every item rests in one of its stable poses, does not touch the others,
    leaves room for the gripper fingers, is visible to the camera and can be
    grasped from the start pose;
  * the arrangement is settled and checked in Gazebo (headless): nothing moves
    by more than a few millimetres;
  * the goal's optimal plan (breadth-first search over the domain) has between
    --min-actions and --max-actions actions, and every one of those actions
    has a collision-free motion plan.

If the robot starts holding an item (init contains robot-gripping), the
world has that item on the table and the solution begins with a setup
motion that picks it up. A problem takes from 20 s to about 5 min.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config  # noqa: E402
from lab_replica.problems.files import write_problem  # noqa: E402
from lab_replica.problems.generator import GenerationError, Generator  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--seed", type=int, default=None, help="seed of the first problem (default: from the clock)")
    ap.add_argument("--out", default=os.path.join(config.workspace_root(), "problems"))
    ap.add_argument("--name", default=None, help="directory name (only with --count 1)")
    ap.add_argument("--basket-items", type=int, default=None, help="items in the basket at the start (default random 0-4)")
    ap.add_argument("--p-holding", type=float, default=0.3, help="probability that the robot starts holding an item")
    ap.add_argument("--min-actions", type=int, default=2)
    ap.add_argument("--max-actions", type=int, default=10)
    ap.add_argument("--randomize-basket", action="store_true", help="random basket position on the board (cell.yaml basket.random_x/y)")
    ap.add_argument("--no-render", action="store_true", help="skip the camera render")
    args = ap.parse_args()

    seed0 = args.seed if args.seed is not None else int(time.time()) % 100000
    done = 0
    for k in range(args.count):
        seed = seed0 + k
        name = args.name if (args.name and args.count == 1) else f"problem_{seed:05d}"
        out_dir = os.path.join(args.out, name)
        print(f"[{k + 1}/{args.count}] {name} (seed {seed})")
        t0 = time.time()
        gen = Generator(seed=seed, randomize_basket=args.randomize_basket)
        try:
            prob = gen.generate(n_basket=args.basket_items, p_holding=args.p_holding,
                                min_actions=args.min_actions, max_actions=args.max_actions)
        except GenerationError as e:
            print(f"  failed: {e}")
            continue
        write_problem(prob, out_dir, name, gen.cell, gen.items, seed=seed, render=not args.no_render)
        done += 1
        print(f"  {len(prob['plan'])} actions: {' '.join(prob['plan'])}")
        print(f"  goal: {' '.join('(' + ' '.join(f) + ')' for f in sorted(prob['goal']))}")
        print(f"  written to {out_dir} ({time.time() - t0:.0f} s)")
    print(f"{done}/{args.count} problems generated")
    return 0 if done == args.count else 1


if __name__ == "__main__":
    sys.exit(main())
