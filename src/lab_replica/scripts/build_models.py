#!/usr/bin/env python3
"""Rebuild the Gazebo models (models/) and worlds/lab_empty.sdf from
config/cell.yaml and config/items.yaml.

    ros2 run lab_replica build_models.py [--out DIR]

replan_problem.py does this too; use this one to look at changed models in
the empty cell (ros2 launch lab_replica sim.launch.py) without replanning.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica.gazebo.build import build_models  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="models directory (default: the package's models/)")
    build_models(ap.parse_args().out)


if __name__ == "__main__":
    main()
