#!/usr/bin/env python3
"""Render the static scene with objects laid out like photo 000047 (no
robot), for side-by-side comparison with the lab photo.

    render_preview.py out.png [--photo task_photo.png]

With --photo it also writes out_overlay.png (50/50 blend) and
out_side.png (side by side).
"""

import argparse
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica import config, objects  # noqa: E402
from lab_replica.gazebo.server import GzServer  # noqa: E402
from lab_replica.gazebo.world import problem_world  # noqa: E402
from lab_replica.planning.placement import Placement  # noqa: E402

LAYOUT_47 = [
    Placement("red-spray-bottle", "upright", 0.120, -0.14, 0.15),
    Placement("green-mug", "upright", 0.450, -0.17, -0.2),
    Placement("green-block", "flat", 0.555, 0.0, 0.1),
    Placement("roll-of-blue-tape", "flat", 0.638, 0.10, 0.0),
    Placement("red-block", "flat", 0.653, 0.22, 0.2),
    Placement("orange-block", "flat", 0.724, 0.14, 0.3),
    Placement("blue-cup-with-yellow-stripe", "upright", 0.820, 0.29, 0.0),
    Placement("yellow-block", "flat", 0.860, 0.22, 0.1),
    Placement("roll-of-black-tape", "flat", 0.930, 0.27, 0.0),
    Placement("red-can", "lying", 0.770, -0.03, math.radians(55)),
    Placement("blue-block", "flat", 0.700, 0.045, 0.3),
    Placement("roll-of-green-tape", "flat", 0.270, -0.22, 0.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--photo")
    args = ap.parse_args()
    cell = config.load_cell()
    items = objects.load_items()
    work = tempfile.mkdtemp(prefix="labprev_")
    frames = os.path.join(work, "frames")
    os.makedirs(frames)
    world = problem_world(cell, items, placements=LAYOUT_47, lift=0.001, world_name="preview", save_dir=frames,
                          gui=False)
    wf = os.path.join(work, "preview.sdf")
    with open(wf, "w") as f:
        f.write(world)
    with GzServer(wf, "preview", rendering=True) as gz:
        gz.wait_until(1.0, timeout=90)
        gz.grab_image(frames, args.out)
    print("wrote", args.out)
    if args.photo:
        import cv2
        a = cv2.imread(args.out)
        b = cv2.imread(args.photo)
        b = cv2.resize(b, (a.shape[1], a.shape[0]))
        root = os.path.splitext(args.out)[0]
        cv2.imwrite(root + "_overlay.png", cv2.addWeighted(a, 0.5, b, 0.5, 0))
        cv2.imwrite(root + "_side.png", cv2.vconcat([b, a]))


if __name__ == "__main__":
    main()
