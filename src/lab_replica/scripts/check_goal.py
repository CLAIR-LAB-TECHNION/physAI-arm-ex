#!/usr/bin/env python3
"""Read the simulated scene back as PDDL facts and check a problem's goal.

    ros2 run lab_replica check_goal.py <problem> [--watch]

<problem> is a name in problems/ (e.g. all_on_table) or a directory.
Needs the simulation (sim.launch.py). Prints the facts and exits 0 when the
goal holds. With --watch it keeps printing whenever the facts change.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from lab_replica import config, objects  # noqa: E402
from lab_replica.execution.scene_state import ModelPoses, facts_from_scene  # noqa: E402
from lab_replica.planning import pddl  # noqa: E402
from lab_replica.robot import rg6  # noqa: E402
from lab_replica.robot.kinematics import ARM_JOINTS, URKinematics  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("problem")
    ap.add_argument("--watch", action="store_true", help="keep printing changes")
    args = ap.parse_args()
    _, goal = pddl.read_problem(os.path.join(config.problem_dir(args.problem), "problem.pddl"))
    cell = config.load_cell()
    basket = objects.load_basket(cell)
    kin = URKinematics(cell["robot"]["ur_type"])
    rclpy.init()
    node = Node("check_goal")
    poses = ModelPoses(node)
    st = {"grip": False, "q": None, "w": 0.1}

    def on_grip(msg):
        st["grip"] = msg.data

    def on_joint_states(msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            st["q"] = np.array([d[j] for j in ARM_JOINTS])
        if "left_finger_slide" in d and "right_finger_slide" in d:
            st["w"] = rg6.width_from_slides(d["left_finger_slide"], d["right_finger_slide"])

    node.create_subscription(Bool, "/rg6/grip_detected", on_grip, 10)
    node.create_subscription(JointState, "/joint_states", on_joint_states, 10)
    if not poses.wait(15.0):
        print("no object poses on /lab/model_poses: is the simulation running?", file=sys.stderr)
        sys.exit(2)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    last = None
    while True:
        facts = facts_from_scene(poses.poses, st["grip"], st["q"], st["w"], cell, basket.r_top, kin)
        ok = pddl.satisfies(facts, goal)
        if facts != last:
            print("state:", " ".join(pddl.fact_str(f) for f in sorted(facts)))
            missing = [g for g in goal if g not in facts]
            print("goal satisfied" if ok else "goal NOT satisfied, missing: " + " ".join(map(pddl.fact_str, missing)))
            last = facts
        if not args.watch:
            break
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
