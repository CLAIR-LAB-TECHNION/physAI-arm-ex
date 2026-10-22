"""Helpers for the recording tools (record_action.py, record_task.py):
starting and stopping a program (the simulation, the skill server) and
reading the simulator's state as PDDL facts."""

import math
import os
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

GRIP_DIST = 0.025     # fingertips within this of an item's box: it is the held one


def start(cmd, env, log, ready, timeout=240):
    f = open(log, "w")
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if p.poll() is not None:
            raise RuntimeError(f"{cmd[0]} exited, see {log}")
        if ready in open(log, errors="ignore").read():
            return p
        time.sleep(2)
    stop(p)
    raise RuntimeError(f"{' '.join(cmd[:4])} not ready after {timeout} s, see {log}")


def stop(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=15)
    except Exception:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()


class Truth:
    """The simulator's state as PDDL facts, plus the item poses."""

    def __init__(self, node):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        from lab_replica import config, objects
        from lab_replica.execution.scene_state import ModelPoses
        from lab_replica.robot.kinematics import ARM_JOINTS, URKinematics
        self.rclpy = rclpy
        self.node = node
        self.cell = config.load_cell()
        self.items = objects.load_items()
        self.basket = objects.load_basket(self.cell)
        self.kin = URKinematics(self.cell["robot"]["ur_type"])
        self.poses = ModelPoses(node)
        self.st = {"grip": False, "q": None, "w": 0.1}
        self.joints = ARM_JOINTS
        node.create_subscription(Bool, "/rg6/grip_detected", lambda m: self.st.__setitem__("grip", m.data), 10)
        node.create_subscription(JointState, "/joint_states", self._js, 10)

    def _js(self, msg):
        from lab_replica.robot import rg6
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in self.joints):
            self.st["q"] = np.array([d[j] for j in self.joints])
        if "left_finger_slide" in d:
            self.st["w"] = rg6.width_from_slides(d["left_finger_slide"], d["right_finger_slide"])

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def read(self):
        """(facts, {item: (xyz, R)})."""
        from lab_replica.planning import pddl
        from lab_replica.robot import rg6
        from lab_replica.robot.collision import object_box
        from lab_replica.transforms import quat_to_matrix
        self.poses.poses.clear()
        self.spin(1.0)
        P = {}
        for it, m in self.items.items():
            if m.model in self.poses.poses:
                xyz, q = self.poses.poses[m.model]
                P[it] = (np.array(xyz), quat_to_matrix(q))
        bxy = self.poses.poses.get("basket", (np.array(self.cell["basket"]["center_xy"] + [0]), None))[0][:2]
        T = self.kin.fk(self.st["q"])
        tip = T[:3, 3] + T[:3, 2] * (rg6.depth_from_width(self.st["w"]) - 0.02)
        facts, held = set(), None
        if self.st["grip"]:
            best = None
            for it, (xyz, R) in P.items():
                b = object_box(self.items[it], xyz, R, it)
                L = (tip - b.c) @ b.R
                d = np.linalg.norm(np.maximum(np.abs(L) - b.h, 0))
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
            r = math.hypot(xyz[0] - bxy[0], xyz[1] - bxy[1])
            if r < self.basket.r_top and xyz[2] < rim + 0.08:
                facts.add(("in-basket", it))
            elif xyz[2] < 0.25:
                facts.add(("on-table", it))
        return frozenset(facts), P

    @staticmethod
    def tilt_change(R, R_before):
        """Angle [deg] by which an item's up axis turned."""
        a, b = R[:, 2], R_before[:, 2]
        return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1, 1))))

    def upright(self, item, R):
        """Smallest tilt [deg] between the item and one of its resting poses."""
        from lab_replica.transforms import rpy_to_matrix
        m = self.items[item]
        best = 180.0
        for rest in m.spec["stable_poses"]:
            Rr = rpy_to_matrix(*m.poses[rest].rpy)
            z_rest = Rr.T @ np.array([0, 0, 1.0])     # world up, in the item frame, when rested
            best = min(best, math.degrees(math.acos(np.clip(np.dot(R @ z_rest, [0, 0, 1.0]), -1, 1))))
        return best


def expected(facts, action):
    from lab_replica.planning import pddl
    for a, s in pddl.successors(facts):
        if a == action:
            return s
    return None
