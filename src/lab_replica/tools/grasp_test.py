#!/usr/bin/env python3
"""Physics test of the grasps: pick each test object and drop it in the basket.

    grasp_test.py world <out.sdf>      write the test world
    grasp_test.py run [items...]       run against a sim started with
                                       ros2 launch lab_replica sim.launch.py world:=<out.sdf>
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config, objects  # noqa: E402
from lab_replica.planning.placement import Placement  # noqa: E402

TEST = [
    Placement("red-block", "flat", 0.42, -0.16, 0.3),
    Placement("roll-of-blue-tape", "flat", 0.58, -0.14, 0.0),
    Placement("red-can", "lying", 0.68, 0.05, 1.2),
    Placement("green-mug", "upright", 0.60, 0.25, 0.5),
    Placement("blue-cup-with-yellow-stripe", "upright", 0.22, -0.20, 0.0),
    Placement("red-spray-bottle", "upright", 0.47, 0.25, 0.4),
    Placement("roll-of-black-tape", "flat", 0.62, -0.24, 0.0),
    Placement("blue-block", "flat", 0.12, -0.22, 0.6),
]


TEST_B = [
    Placement("red-can", "upright", 0.45, -0.15, 0.0),
    Placement("green-mug", "upright", 0.62, 0.20, 0.5),
    Placement("roll-of-green-tape", "flat", 0.30, -0.22, 0.0),
    Placement("yellow-block", "flat", 0.68, -0.10, 0.4),
    Placement("orange-block", "flat", 0.56, 0.11, 0.9),
    Placement("green-block", "flat", 0.75, 0.08, 0.2),
    Placement("blue-cup-with-yellow-stripe", "upright", 0.60, -0.20, 0.0),
    Placement("red-spray-bottle", "upright", 0.47, 0.25, 0.4),
]
if os.environ.get("LAYOUT") == "B":
    TEST = TEST_B


def write_world(out):
    from lab_replica.gazebo.world import problem_world
    scene = config.load_cell()
    objs = objects.load_items()
    with open(out, "w") as f:
        f.write(problem_world(scene, objs, placements=TEST))
    print("wrote", out)


def run(items):
    import rclpy

    from lab_replica.execution.executor import Executor
    from lab_replica.execution.scene_state import ModelPoses
    from lab_replica.planning.skills import Skills
    from lab_replica.transforms import quat_to_matrix, rpy_to_matrix

    scene = config.load_cell()
    objs = objects.load_items()
    sk = Skills(scene, objs, objects.load_basket(scene))
    rclpy.init()
    ex = Executor()
    mp = ModelPoses(ex)
    ex.wait_ready()
    mp.wait()
    q = ex.current()
    bx, by = scene["basket"]["center_xy"]
    slots = [(bx - 0.04, by - 0.04), (bx + 0.045, by + 0.03), (bx - 0.03, by + 0.05), (bx + 0.04, by - 0.05),
             (bx, by), (bx - 0.06, by), (bx + 0.06, by), (bx, by + 0.06)]
    results = {}
    world = {p.item: p for p in TEST}
    for k, p in enumerate(TEST):
        if items and p.item not in items:
            continue
        model = config.model_name(p.item)
        print(f"== {p.item}")
        xyz0, quat0 = mp.get(model)
        z0 = xyz0[2]
        R_start = quat_to_matrix(quat0)
        steps, q, held = sk.plan_pick(q, p.item, world)
        try:
            ex.run_steps(steps)
        except Exception as e:  # noqa: BLE001
            print("   FAILED:", e)
            results[p.item] = f"pick failed: {e}"
            steps, q = sk.plan_home(ex.current(), {k: v for k, v in world.items() if k != p.item})
            ex.run_steps(steps)
            q = ex.current()
            continue
        xyz1, quat1 = mp.get(model)
        z1 = xyz1[2]
        R1 = quat_to_matrix(quat1)
        tilt1 = math.degrees(math.acos(max(-1.0, min(1.0, float((R1 @ R_start.T)[2, 2])))))
        print(f"   lifted by {z1 - z0:.3f} m, tilted {tilt1:.0f} deg in the grasp")
        if os.environ.get("SNAP"):
            import subprocess
            subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "scripts",
                                                         "capture_camera.py"), f"{os.environ['SNAP']}_{model}.png"])
        del world[p.item]
        if os.environ.get("TO") == "table":
            region = dict(x_range=(0.15, 0.80), y_range=(-0.25, 0.28),
                          keepout=[(0.0, 0.0, 0.17), (bx, by, sk.basket.r_top + 0.03)])
            tgt, steps, q = sk.find_spot(q, held, "table", world, region=region, prefer=(p.x + 0.12, p.y))
        else:
            tgt, steps, q = sk.find_spot(q, held, "basket", world, prefer=slots[k % len(slots)])
        world[p.item] = tgt
        ex.run_steps(steps)
        import time
        t0 = time.time()
        while time.time() - t0 < 1.5:
            mp.get(model)
        xyz, quat = mp.get(model)
        R = quat_to_matrix(quat)
        if os.environ.get("SNAP"):
            import subprocess
            subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "scripts",
                                                         "capture_camera.py"), f"{os.environ['SNAP']}_{model}_placed.png"],
                           capture_output=True)
        r = math.hypot(xyz[0] - bx, xyz[1] - by)
        R0 = rpy_to_matrix(*objs[p.item].poses[p.rest].rpy)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((R @ R0.T)[2, 2])))))
        if os.environ.get("TO") == "table":
            err = math.hypot(xyz[0] - tgt.x, xyz[1] - tgt.y)
            ok = tilt < 5 and err < 0.02
            print(f"   on table: {ok} (position error {err * 1000:.0f} mm, tilt {tilt:.1f} deg)")
        else:
            ok = r < sk.basket.r_bottom and xyz[2] < 0.15
            print(f"   in basket: {ok} (r={r:.3f}, z={xyz[2]:.3f}, tilt {tilt:.0f} deg)")
        results[p.item] = "ok" if ok and z1 - z0 > 0.1 else f"FAIL lift {z1 - z0:.3f}, tilt {tilt:.0f}"
    steps, q = sk.plan_home(ex.current(), world)
    ex.run_steps(steps)
    print("\nRESULTS")
    for k, v in results.items():
        print(f"  {k:30s} {v}")
    rclpy.shutdown()


def dry(items):
    import time
    import numpy as np
    from lab_replica.planning.skills import Skills
    scene = config.load_cell()
    objs = objects.load_items()
    sk = Skills(scene, objs, objects.load_basket(scene))
    q = np.array(scene["robot"]["home_joints"])
    world = {p.item: p for p in TEST}
    for p in TEST:
        if items and p.item not in items:
            continue
        t0 = time.time()
        try:
            steps, q2, held = sk.plan_pick(q, p.item, world)
            w2 = {k: v for k, v in world.items() if k != p.item}
            tgt, st2, q3 = sk.find_spot(q2, held, "basket", w2)
            dur = sum(s["times"][-1] for s in steps + st2 if s["kind"] == "traj")
            print(f"{p.item:30s} OK  plan {time.time() - t0:.1f} s, motion {dur:.0f} s, drop {st2[2].get('drop')}")
            world = dict(w2)
            world[p.item] = tgt
            q = q3
        except Exception as e:  # noqa: BLE001
            print(f"{p.item:30s} ERR {e}  ({time.time() - t0:.1f} s)")


if __name__ == "__main__":
    if sys.argv[1] == "dry":
        dry(sys.argv[2:])
    elif sys.argv[1] == "world":
        write_world(sys.argv[2])
    else:
        run(sys.argv[2:])
