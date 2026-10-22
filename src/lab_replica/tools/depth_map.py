#!/usr/bin/env python3
"""Rebuild what the robot saw from saved camera frames, offline.

    python3 src/lab_replica/tools/depth_map.py runs/<run>/frame_*.npz [--out map.png] [--pick ITEM]

The frames are those saved with --save-frames (action or goal console,
skill_server.py): colour, depth, intrinsics, camera pose and arm joints.
Runs SAM 3 and the pose fitting on them, builds the obstacle map of the
motion planner (autonomy/depth_world.py) and writes its top view (--out).
With --pick ITEM it also plans the grasp of ITEM from the joints of the
last frame and prints what blocks it. Needs the .venv (SAM 3), no ROS
graph.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config, objects  # noqa: E402
from lab_replica.autonomy.depth_world import DepthWorld  # noqa: E402
from lab_replica.autonomy.obstacles import PerceivedPickPlace  # noqa: E402
from lab_replica.perception.rgbd import Frame  # noqa: E402
from lab_replica.perception.scene import ScenePerception, draw, load_perception_config  # noqa: E402
from lab_replica.planning.motion import PlanError  # noqa: E402


def load_frame(path):
    d = np.load(path)
    f = Frame(d["rgb"], d["depth"], d["K"], 0.0, T_world_cam=d["T"])
    f.q = d["q"]
    return f


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", nargs="+", help="frame_*.npz files (the first is the main view)")
    ap.add_argument("--out", default="depth_map.png", help="top view of the map (PNG)")
    ap.add_argument("--pick", help="plan the grasp of this item from the last frame's joints")
    args = ap.parse_args()

    cell, items = config.load_cell(), objects.load_items()
    basket = objects.load_basket(cell)
    cfg = load_perception_config()
    frames = [load_frame(p) for p in args.frames]
    perception = ScenePerception(cell, items, basket, cfg=cfg)
    scene = perception.observe(frames, [1.0] + [0.9] * (len(frames) - 1))
    print(f"found {len(scene.detections)} items: {', '.join(sorted(scene.detections))}")
    for it, d in sorted(scene.detections.items()):
        p = d.estimate.placement
        print(f"  {it:28s} ({p.x:.3f}, {p.y:.3f}) yaw {np.degrees(p.yaw):4.0f} deg, {p.rest} on the {p.support}, "
              f"score {d.score:.2f}")
    depth = DepthWorld(frames, cell, items, basket, scene.basket_xy, scene.detections,
                       max_item_height=cfg["max_item_height"], robot_radius=cfg["robot_radius"])
    cv2.imwrite(args.out, depth.image())
    cv2.imwrite(os.path.splitext(args.out)[0] + "_sam.jpg", draw(frames[0], scene.detections, scene.basket_xy))
    print(f"wrote {args.out}")
    for o in depth.blobs():
        c = o.polygon.mean(axis=0)
        print(f"  {'unseen space' if o.unseen else 'unrecognised'} at ({c[0]:.3f}, {c[1]:.3f}), "
              f"{o.z_top * 100:.0f} cm high")
    if args.pick:
        sk = PerceivedPickPlace(cell, items, basket, basket_xy=scene.basket_xy, depth=depth)
        world = scene.world()
        try:
            steps, q_end, held = sk.plan_pick(frames[-1].q, args.pick, world, max_candidates=32)
            print(f"pick of {args.pick}: OK ({len(steps)} steps, width {held.width * 1000:.0f} mm)")
        except PlanError as e:
            print(f"pick of {args.pick}: {e}")
        try:
            home, q_home = sk.plan_home(frames[-1].q, world)
            steps, q_end, held = sk.plan_pick(q_home, args.pick, world, max_candidates=32)
            print(f"pick of {args.pick} from home: OK")
        except PlanError as e:
            print(f"pick of {args.pick} from home: {e}")


if __name__ == "__main__":
    main()
