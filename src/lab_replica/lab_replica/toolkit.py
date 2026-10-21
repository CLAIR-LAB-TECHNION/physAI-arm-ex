"""Everything the exercise (lab_replica/exercise/) uses from the rest of the
code: data types, small geometric helpers, the robot's kinematics, and the
steps of a robot plan. Nothing here needs ROS.

Data types
    Placement     where an item rests: item, rest (resting pose), x, y, yaw, support, z_support
    Estimate      an item's estimated Placement, with plausibility, n_points, top_z, notes
    Detection     an item found in a frame: item, score, sam_score, estimate, mask, view, frame
    ImageGrasp    a top-down grasp in an image: pixel (u, v), closing angle, width in pixels
    Grasp         a top-down grasp in the world: pose of rg6_base_link, width, tips_z, top_z
    Held          how the gripper holds an item (see Held below)
    Box           an oriented box: name, centre c, rotation R, half sizes h
    Cylinder      a vertical cylinder: name, centre c, radius r, half height h
    PlanError     raised when no motion is found

Perception helpers (perception/estimate.py)
    circle_fit, rect_fit, robust_top, top_layer, gauss, fit_mug, fit_spray_bottle, mask_overlap

Robot and geometry
    ARM_JOINTS, ik_all(T), fk(q), joint_limits(), gripper_pose(x, y, z, yaw),
    rg6 (depth_from_width, free_height, MAX_WIDTH, ...), finger_footprints,
    object_box, placement_pose, polygons_intersect, polygon_inside_rect,
    rpy_to_matrix, wrap

Plans
    trajectory(label, path, ...), gripper_step(label, width, force, ...)
"""

import functools
import math
from dataclasses import dataclass

import numpy as np

from . import config
from .perception.estimate import (Estimate, circle_fit, fit_mug, fit_spray_bottle, gauss, rect_fit, robust_top,
                                  top_layer)
from .planning.geometry2d import polygon_inside_rect, polygons_intersect
from .planning.motion import PlanError, Planner, gripper_pose
from .planning.placement import Placement, placement_pose
from .planning.skills import Held
from .robot import rg6
from .moveit_client import Cylinder
from .robot.collision import Box, object_box
from .robot.kinematics import ARM_JOINTS, URKinematics
from .transforms import rpy_to_matrix, wrap

__all__ = ["ARM_JOINTS", "Box", "Cylinder", "Detection", "Estimate", "Grasp", "Held", "ImageGrasp", "PlanError", "Placement",
           "box_in_world", "circle_fit", "finger_footprints", "fit_mug", "fit_spray_bottle", "fk", "gauss",
           "gripper_pose", "gripper_step", "ik_all", "joint_limits", "mask_overlap", "object_box",
           "placement_pose", "polygon_inside_rect", "polygons_intersect", "rect_fit", "rg6", "robust_top",
           "rpy_to_matrix", "top_layer", "trajectory", "wrap"]


@dataclass
class ImageGrasp:
    """A top-down grasp proposed in an image."""
    u: float            # the pixel the fingers close around (column ...
    v: float            # ... and row)
    angle: float        # the direction the fingers close along, in the image: radians from +u towards +v
    width_px: float     # the item's width across, in pixels


@dataclass(eq=False)
class Grasp:
    """A top-down grasp in the world."""
    pose: np.ndarray    # 4x4 world pose of rg6_base_link (the mounting face) at the grasp
    width: float        # the item's width between the pads [m]
    tips_z: float       # the fingertips' height at the grasp (world z)
    top_z: float        # the item's top (world z)


def __getattr__(name):
    # Detection lives with the scene perception, which imports the exercise:
    # imported on first use to keep the imports acyclic.
    if name == "Detection":
        from .perception.scene import Detection
        return Detection
    raise AttributeError(name)


def mask_overlap(a, b):
    """The intersection of two boolean masks over the smaller one's area."""
    inter = np.logical_and(a, b).sum()
    return inter / max(1, min(a.sum(), b.sum()))


# ---- the robot --------------------------------------------------------------------------


@functools.lru_cache(maxsize=1)
def _kinematics():
    return URKinematics(config.load_cell()["robot"]["ur_type"])


@functools.lru_cache(maxsize=1)
def _planner():
    return Planner(config.load_cell())


def ik_all(T):
    """All inverse kinematics solutions (arrays of the six joint angles, each
    in [-pi, pi]) that put rg6_base_link at the world pose T; [] if none."""
    return _kinematics().ik_all(T)


def fk(q):
    """The world pose (4x4) of rg6_base_link at the arm joints q."""
    return _kinematics().fk(q)


def joint_limits():
    """{joint name: (lower, upper)} of the arm joints [rad]."""
    lim = _kinematics().chain.limits
    return {j: lim.get(j, (-2 * math.pi, 2 * math.pi)) for j in ARM_JOINTS}


def box_in_world(box, q):
    """A box given in the gripper's frame (e.g. held_box) in the world, with the arm at q."""
    T = fk(q)
    return Box(box.name, T[:3, :3] @ box.c + T[:3, 3], T[:3, :3] @ box.R, box.h)


def finger_footprints(x, y, yaw, open_width):
    """The footprints (2D rectangles (4, 2), world) of the two fingers of a
    top-down gripper at (x, y) with yaw `yaw`, opened to open_width."""
    d = np.array([math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)])   # the closing direction
    t = np.array([-d[1], d[0]])
    out = []
    for side in (-1, 1):
        c = np.array([x, y]) + side * d * (open_width / 2 + rg6.FINGER_THICKNESS / 2)
        hw, hd = rg6.FINGER_HALF_WIDTH + 0.003, rg6.FINGER_THICKNESS / 2 + 0.003
        out.append(np.array([c + a * hd * d + b * hw * t for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]))
    return out


# ---- plans -------------------------------------------------------------------------------


def trajectory(label, path, slow=False, slow_last=None, slow_first=None):
    """A step that moves the arm along a joint path (a list of 6-arrays),
    timed with a smooth (minimum-jerk) profile within the robot's speed
    limits. slow: all of it at approach speed; slow_last / slow_first: the
    last / first so many metres of the gripper's way (a descent, a lift)."""
    P = _planner()
    path = [np.asarray(q, float) for q in path]
    if slow_last or slow_first:
        pts = [P.kin.fk(q)[:3, 3] for q in path]
        n = len(path)
        if slow_last:
            k = next((i for i in range(n) if np.linalg.norm(pts[i] - pts[-1]) <= slow_last + 1e-9), n - 1)
            fast, slow_part = path[:k + 1], path[k:]
        else:
            k = next((i for i in range(n) if np.linalg.norm(pts[i] - pts[0]) >= slow_first - 1e-9), n - 1)
            slow_part, fast = path[:k + 1], path[k:]
        t_fast = P.time_path(fast) if len(fast) > 1 else [0.0]
        t_slow = P.time_path(slow_part, speed=P.V_SLOW) if len(slow_part) > 1 else [0.0]
        if slow_last:
            times = t_fast + [t_fast[-1] + t for t in t_slow[1:]]
        else:
            times = t_slow + [t_slow[-1] + t for t in t_fast[1:]]
    else:
        times = P.time_path(path, speed=P.V_SLOW if slow else None)
    return dict(kind="traj", label=label, points=[list(map(float, q)) for q in path], times=times)


def gripper_step(label, width, force, expect_grip=False, **extra):
    """A step that moves the RG6's fingers to `width` (m) with at most
    `force` (N). expect_grip: the step fails unless the fingers stop on
    something (a grasp)."""
    return dict(kind="gripper", label=label, width=float(width), force=float(force), expect_grip=bool(expect_grip),
                **extra)
