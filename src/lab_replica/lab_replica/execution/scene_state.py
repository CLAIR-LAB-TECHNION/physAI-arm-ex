"""The simulated scene read back as PDDL facts, for checking a goal.

Object poses come from Gazebo through the /lab/model_poses bridge
(sim.launch.py); the grip state from /rg6/grip_detected.
"""

import math
import time

import numpy as np
import rclpy
from tf2_msgs.msg import TFMessage

from .. import config
from ..planning import pddl
from ..robot import rg6

IN_BASKET_ABOVE_RIM = 0.08   # an item's centre this far above the rim still counts as in the basket
ON_TABLE_BELOW = 0.25        # an item's centre below this height (and not in the basket) is on the table
GRIPPED_WITHIN = 0.08        # an item's centre this close to the fingertips is the gripped one


class ModelPoses:
    """Keeps the latest world pose of every model, keyed by model name."""

    def __init__(self, node, topic="/lab/model_poses"):
        self.node = node
        self.poses = {}
        node.create_subscription(TFMessage, topic, self._cb, 10)

    def _cb(self, msg):
        for tf in msg.transforms:
            name = tf.child_frame_id
            # Top-level models come with the world as parent; links of nested
            # entities carry their parent model's name as frame_id.
            if tf.header.frame_id not in ("", "lab", "world") and name in self.poses and "::" not in name:
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            self.poses[name] = (np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w]))

    def wait(self, timeout=10.0):
        t0 = time.time()
        while not self.poses and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return bool(self.poses)

    def get(self, model):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.poses.get(model)


def facts_from_scene(poses, grip, q_arm, width, cell, basket_r, kin):
    """PDDL facts of the scene.

      (robot-gripping X)    the RG6 reports a grip and X is the item between the fingertips
      (in-basket X)         X's centre is inside the basket rim (horizontally), at most
                            IN_BASKET_ABOVE_RIM above it
      (on-table X)          otherwise, when X's centre is low (it is not lifted)
      (robot-gripper-empty) when nothing is gripped
    """
    b = cell["basket"]
    bxy = poses.get("basket", (np.array(b["center_xy"] + [0.0]), None))[0]
    rim = cell["table"]["surface_z"] + b["height"]
    tip = None
    if q_arm is not None:
        T = kin.fk(q_arm)
        tip = T[:3, 3] + T[:3, 2] * (rg6.depth_from_width(width) - 0.03)
    facts = set()
    gripped = None
    if grip and tip is not None:
        best = None
        for it in pddl.ITEMS:
            m = config.model_name(it)
            if m in poses:
                d = np.linalg.norm(poses[m][0] - tip)
                if best is None or d < best[0]:
                    best = (d, it)
        if best and best[0] < GRIPPED_WITHIN:
            gripped = best[1]
            facts.add(("robot-gripping", gripped))
    if gripped is None:
        facts.add(pddl.EMPTY)
    for it in pddl.ITEMS:
        m = config.model_name(it)
        if it == gripped or m not in poses:
            continue
        xyz = poses[m][0]
        r = math.hypot(xyz[0] - bxy[0], xyz[1] - bxy[1])
        if r < basket_r and xyz[2] < rim + IN_BASKET_ABOVE_RIM:
            facts.add(("in-basket", it))
        elif xyz[2] < ON_TABLE_BELOW:
            facts.add(("on-table", it))
    return facts
