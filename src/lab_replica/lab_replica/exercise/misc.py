"""New items: items that are not in config/items.yaml, added while a console
runs.

    new-item: purple-block prompt="purple block"   ->  new_item(robot, "purple-block", "purple block")
    new-item: lid shape=round                      ->  new_item(robot, "lid", "lid", "round")
    forget-item: purple-block                      ->  forget_item(robot, "purple-block")

new_item returns (spec, message) or raises ActionError / PlanError, like
the skills (skills.py); after it succeeds the robot returns home by itself.
Added items last until the console quits.

Besides what skills.py describes, the robot (autonomy/executive.py) has:
    robot.segment(frame, prompt)     SAM 3's masks of a text prompt in `frame`, best first: each
                                     with .score and .mask (shown in the camera window)
    robot.points(frame, mask, erode_px=None)
                                     the world points (N, 3) of a mask's pixels, above the table and
                                     away from the robot (eroded by perception.yaml's mask_erode_px;
                                     erode_px=0: not eroded)
    robot.points_around(frame, mask, model, others=())
                                     the world points around a mask, table included (what
                                     estimate_item fits the model to), without the pixels of the
                                     masks `others`
    robot.add_item(name, spec)       know `name` from now on: its model and its SAM 3 prompts
    robot.remove_item(name)          forget it
"""

import math  # noqa: F401
import time  # noqa: F401

import numpy as np  # noqa: F401

from ..toolkit import (ActionError, PlanError, estimate_item, make_item, mask_overlap, measure_item, rg6,  # noqa: F401
                       show_new_item)
from . import state_estimation as se  # noqa: F401

MIN_POINTS = None        # TODO: depth points a mask needs to count as an item


def new_item(robot, name, prompt, shape=None):
    """Find an item that is not known yet on the table by its text prompt,
    measure it and add it as `name`. Returns (spec, message).

    shape  "box" or "round", or None to tell from its points

    The spec is a dict as in config/items.yaml for objects/measured.py:
        {"kind": "measured", "shape": "box", "size": [length, width, height],
         "mass": 0.1, "color": [r, g, b] (0 to 1), "stable_poses": ["flat"],
         "prompts": [prompt]}
    for a box (length >= width), or "shape": "round", "size": [diameter,
    height], "stable_poses": ["upright"] for a round item.

    Hints:
      * Start with boxes (shape "box"), then round items.
      * The gripper must be empty. Look (robot.survey), then ask SAM 3 for the
        prompt in the frames (robot.segment). A mask that a known item
        already has (toolkit.mask_overlap with the scene's detections), or one
        in the basket, is not the new item.
      * Its points (robot.points; not eroded, or it measures small) give its
        size: toolkit.measure_item(pts, table_z, shape) returns (shape, size,
        notes). A closer look (se.close_views) measures better.
      * Refuse what the RG6 cannot grasp (rg6.MAX_WIDTH) or what is taller
        than robot.cfg["max_item_height"].
      * robot.add_item(name, spec) makes the robot know it. Check that its
        model (toolkit.make_item) explains the points it was made from
        (toolkit.estimate_item, the plausibility), and show it
        (toolkit.show_new_item)."""
    # TODO
    raise NotImplementedError("misc.new_item (exercise/misc.py)")


def forget_item(robot, name):
    """Forget the item `name` added with new_item."""
    # TODO
    raise NotImplementedError("misc.forget_item (exercise/misc.py)")
