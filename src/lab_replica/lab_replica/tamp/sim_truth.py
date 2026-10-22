"""The simulator's true state as PDDL facts, for checking a goal after a run.

Only for evaluation: the robot never sees it. Item poses come from Gazebo
(/lab/model_poses, sim.launch.py), the grip from /rg6/grip_detected, the
fingertips from the arm's joint angles. The rules (as in
tools/sim_run.py Truth):

  (robot-gripping X)     the RG6 reports a grip and X's box is within 2.5 cm of the fingertips
  (in-basket X)          X's centre is inside the basket rim, at most 8 cm above it
  (on-table X)           otherwise, when X's centre is below 25 cm
  (robot-gripper-empty)  nothing is gripped
"""

import math
import time

import numpy as np

GRIP_DIST = 0.025
ABOVE_RIM = 0.08
ON_TABLE_BELOW = 0.25


class SimTruth:
    def __init__(self, node):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        from .. import config, objects
        from ..execution.scene_state import ModelPoses
        from ..robot.kinematics import ARM_JOINTS, URKinematics
        self.node = node
        self.cell = config.load_cell()
        self.items = objects.load_items()
        self.basket = objects.load_basket(self.cell)
        self.kin = URKinematics(self.cell["robot"]["ur_type"])
        self.poses = ModelPoses(node)
        self.joints = ARM_JOINTS
        self.st = {"grip": False, "q": None, "w": 0.1}
        node.create_subscription(Bool, "/rg6/grip_detected", lambda m: self.st.__setitem__("grip", m.data), 10)
        node.create_subscription(JointState, "/joint_states", self._js, 10)

    def _js(self, msg):
        from ..robot import rg6
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in self.joints):
            self.st["q"] = np.array([d[j] for j in self.joints])
        if "left_finger_slide" in d and "right_finger_slide" in d:
            self.st["w"] = rg6.width_from_slides(d["left_finger_slide"], d["right_finger_slide"])

    def spin(self, seconds):
        import rclpy
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def available(self, timeout=5.0):
        """Whether the simulator's poses are published (False on the real robot)."""
        t0 = time.time()
        while not self.poses.poses and time.time() - t0 < timeout:
            self.spin(0.1)
        return bool(self.poses.poses) and self.st["q"] is not None

    def facts(self):
        from ..planning import pddl
        from ..robot import rg6
        from ..robot.collision import object_box
        from ..transforms import quat_to_matrix
        self.poses.poses.clear()
        self.spin(1.0)
        P = {}
        for it, m in self.items.items():
            if m.model in self.poses.poses:
                xyz, q = self.poses.poses[m.model]
                P[it] = (np.array(xyz), quat_to_matrix(q))
        bxy = self.poses.poses.get("basket", (np.array(self.cell["basket"]["center_xy"] + [0]), None))[0][:2]
        facts, held = set(), None
        if self.st["grip"] and self.st["q"] is not None:
            T = self.kin.fk(self.st["q"])
            tip = T[:3, 3] + T[:3, 2] * (rg6.depth_from_width(self.st["w"]) - 0.02)
            best = None
            for it, (xyz, R) in P.items():
                b = object_box(self.items[it], xyz, R, it)
                d = np.linalg.norm(np.maximum(np.abs((tip - b.c) @ b.R) - b.h, 0))
                if best is None or d < best[0]:
                    best = (d, it)
            if best and best[0] < GRIP_DIST:
                held = best[1]
                facts.add(("robot-gripping", held))
        if held is None:
            facts.add(pddl.EMPTY)
        rim = self.cell["table"]["surface_z"] + self.basket.height
        for it, (xyz, R) in P.items():
            if it == held:
                continue
            if math.hypot(xyz[0] - bxy[0], xyz[1] - bxy[1]) < self.basket.r_top and xyz[2] < rim + ABOVE_RIM:
                facts.add(("in-basket", it))
            elif xyz[2] < ON_TABLE_BELOW:
                facts.add(("on-table", it))
        return frozenset(facts)
