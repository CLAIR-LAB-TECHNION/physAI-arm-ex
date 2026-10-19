#!/usr/bin/env python3
"""Pick one object, log its tilt while closing and lifting, and put it back.

    single_grasp.py world <item> <rest> <x> <y> <yaw> <out.sdf>
    single_grasp.py run <item> <rest> <x> <y> <yaw>
"""

import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config, objects  # noqa: E402
from lab_replica.planning.placement import Placement  # noqa: E402


def main():
    mode, item, rest = sys.argv[1], sys.argv[2], sys.argv[3]
    x, y, yaw = map(float, sys.argv[4:7])
    p = Placement(item, rest, x, y, yaw)
    scene = config.load_cell()
    objs = objects.load_items()
    if mode == "world":
        from lab_replica.gazebo.world import problem_world
        with open(sys.argv[7], "w") as f:
            f.write(problem_world(scene, objs, placements=[p]))
        return
    import numpy as np
    import rclpy

    from lab_replica.execution.executor import Executor
    from lab_replica.execution.scene_state import ModelPoses
    from lab_replica.planning.skills import Skills
    from lab_replica.transforms import quat_to_matrix, rpy_to_matrix

    sk = Skills(scene, objs, objects.load_basket(scene))
    rclpy.init()
    ex = Executor()
    mp = ModelPoses(ex)
    ex.wait_ready()
    mp.wait()
    model = config.model_name(item)
    R0 = rpy_to_matrix(0, 0, yaw) @ rpy_to_matrix(*objs[item].poses[rest].rpy)
    log = []
    stop = threading.Event()

    def sample():
        while not stop.is_set():
            pose = mp.poses.get(model)
            if pose is not None:
                R = quat_to_matrix(pose[1])
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((R @ R0.T)[2, 2])))))
                log.append((time.time(), pose[0].copy(), tilt))
            time.sleep(0.05)

    th = threading.Thread(target=sample, daemon=True)
    th.start()
    steps, q, held = sk.plan_pick(ex.current(), item, {item: p})
    marks = []
    for st in steps:
        marks.append((time.time(), st["label"]))
        ex.run_steps([st], log=lambda *_: None)
    time.sleep(1.0)
    stop.set()
    th.join()
    marks.append((time.time(), "end"))
    print(f"held {held.width * 1000:.1f} mm, tip_z {held.tip_z:.3f}")
    for (t0, lab), (t1, _) in zip(marks[:-1], marks[1:]):
        seg = [s for s in log if t0 <= s[0] < t1]
        if seg:
            print(f"  {lab:18s} tilt {seg[0][2]:5.1f} -> {seg[-1][2]:5.1f} deg, max {max(s[2] for s in seg):5.1f}; "
                  f"z {seg[0][1][2]:.3f} -> {seg[-1][1][2]:.3f}, xy shift {np.linalg.norm(seg[-1][1][:2] - seg[0][1][:2]) * 1000:.1f} mm")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
