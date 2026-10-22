"""What every item model has, and the shape helpers the item files share.

An item model (one class per kind, in can.py, mug.py, ...) defines
  * `shapes`: collision primitives (boxes and cylinders, which Gazebo's DART
    engine handles robustly) in the item's own frame;
  * `poses`: the stable resting poses the item may take on a support, each
    with its top-down grasps;
  * `visual(texture_dir)`: the meshes Gazebo draws.
Mass and inertia follow from `shapes` and the measured mass.

Item frame: z up when the item stands upright, origin at the centre of its
bounding box (for the mug and the spray bottle: on the body axis). A resting
pose rotates the item frame by `rpy` and lifts it by `z` so it sits on the
support; a placement then adds a yaw about the vertical and an (x, y).
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from .. import config
from ..robot import rg6
from ..transforms import matrix_to_rpy, rpy_to_matrix


@dataclass
class Shape:
    """A collision primitive in the item frame."""

    kind: str                       # 'box' or 'cylinder'
    size: Tuple[float, ...]         # box: (x, y, z); cylinder: (radius, length)
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def volume(self):
        if self.kind == "box":
            return float(np.prod(self.size))
        r, h = self.size
        return math.pi * r * r * h

    def inertia_local(self, m):
        if self.kind == "box":
            x, y, z = self.size
            return np.diag([m * (y * y + z * z), m * (x * x + z * z), m * (x * x + y * y)]) / 12.0
        r, h = self.size
        a = m * (3 * r * r + h * h) / 12.0
        return np.diag([a, a, m * r * r / 2.0])


@dataclass
class Grasp:
    """A family of top-down grasps for one resting pose.

    width:  item width between the pads.
    tip_z:  height of the fingertip ends above the support when grasped.
    yaws:   closing directions (rad) in the rested frame; the closing
            direction is the line the two fingertips move along.
    center: (x, y) of the grasp axis in the rested frame; (0, 0) means
            "through the centre of mass" (filled in by ItemModel).
    continuous: any closing direction is valid (`yaws` is then ignored).
    """

    width: float
    tip_z: float
    yaws: List[float] = field(default_factory=lambda: [0.0])
    continuous: bool = False
    center: Tuple[float, float] = (0.0, 0.0)


@dataclass
class RestPose:
    """A way the item can rest on a support."""

    name: str
    rpy: Tuple[float, float, float]  # item frame -> rested frame (before the yaw)
    z: float                         # height of the item origin above the support
    top: float                       # height of the item's top above the support
    footprint: np.ndarray            # (N, 2) convex polygon in the rested frame
    grasps: List[Grasp]


class ItemModel:
    """Base class of the item kinds. Subclasses implement build() and visual()."""

    def __init__(self, name, spec):
        self.name = name                       # PDDL name, e.g. red-can
        self.model = config.model_name(name)   # Gazebo model name, e.g. red_can
        self.spec = spec                       # its entry in items.yaml
        self.kind = spec["kind"]
        self.mass = float(spec["mass"])
        self.shapes = []
        self.poses = {}
        self.build(spec)
        self._mass_properties()
        self._center_grasps_on_com()

    def build(self, spec):
        """Fill self.shapes and self.poses from the measurements."""
        raise NotImplementedError

    def visual(self, texture_dir):
        """List of meshgen.Mesh; textures are written to texture_dir."""
        raise NotImplementedError

    def _mass_properties(self):
        """Centre of mass and inertia, the mass spread over the shapes by volume."""
        vols = np.array([sh.volume() for sh in self.shapes])
        masses = self.mass * vols / vols.sum()
        com = np.zeros(3)
        for sh, m in zip(self.shapes, masses):
            com += m * np.array(sh.xyz)
        com /= self.mass
        inertia = np.zeros((3, 3))
        for sh, m in zip(self.shapes, masses):
            R = rpy_to_matrix(*sh.rpy)
            d = np.array(sh.xyz) - com
            inertia += R @ sh.inertia_local(m) @ R.T + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        self.com = com
        self.inertia = inertia

    def _center_grasps_on_com(self):
        """Grasps without an explicit centre go through the centre of mass,
        so gravity does not twist the item between the pads."""
        for rest in self.poses.values():
            c = rpy_to_matrix(*rest.rpy) @ self.com
            for g in rest.grasps:
                if g.center == (0.0, 0.0):
                    g.center = (float(c[0]), float(c[1]))

    def footprint_world(self, pose_name, x, y, yaw):
        """Footprint polygon on the support for a resting pose placed at (x, y, yaw)."""
        fp = self.poses[pose_name].footprint
        c, s = math.cos(yaw), math.sin(yaw)
        return fp @ np.array([[c, -s], [s, c]]).T + np.array([x, y])


# ---- grasp heights ---------------------------------------------------------

# Clearance between the fingertip ends and the support. Small, so the pads
# catch low items near their base, but enough not to scrape.
TIP_CLEARANCE = 0.008
# The pads are about 5 cm long; on tall items they cover the top 5.8 cm at most.
PAD_REACH = 0.058
PAD_LENGTH = 0.050
# Gap kept between the item's top and the knuckles, which reach into the space
# between the pads rg6.free_height(width) above the fingertips (about 6 cm).
KNUCKLE_CLEARANCE = 0.004


def max_reach(width):
    """Largest height of the item's top above the fingertips for a grasp `width` wide."""
    return rg6.free_height(width) - KNUCKLE_CLEARANCE


def top_grasp_height(top, width):
    """Fingertip height for holding a tall item near its top."""
    return max(TIP_CLEARANCE, top - min(PAD_REACH, max_reach(width)))


def grasp_height(top, width, com_z=None):
    """Fingertip height for a top-down grasp of an item `top` tall.

    The pads (5 cm long) are centred on the centre of mass when the knuckles
    still clear the item's top; otherwise they go as low as the knuckles
    allow. They always overlap the item by at least 3 cm."""
    lo = max(TIP_CLEARANCE, top - max_reach(width))
    hi = max(lo, top - 0.030)
    want = (com_z if com_z is not None else top / 2) - PAD_LENGTH / 2
    return min(hi, max(lo, want))


# ---- shape helpers ---------------------------------------------------------

EIGHT_YAWS = [k * math.pi / 4 for k in range(8)]   # grasp directions of round items (face normals)


def circle(r, n=24, cx=0.0, cy=0.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def rect(sx, sy, cx=0.0, cy=0.0):
    return np.array([[cx - sx / 2, cy - sy / 2], [cx + sx / 2, cy - sy / 2],
                     [cx + sx / 2, cy + sy / 2], [cx - sx / 2, cy + sy / 2]])


def round_prism(d, h, z=0.0, rpy=(0.0, 0.0, 0.0), base_disk=True, sides=16):
    """Collision shapes of a round item: a regular prism with `sides` faces
    (flat-to-flat size d, height h, centred at height z) as the union of
    sides / 2 boxes.

    Used instead of cylinders: DART gives a flat pad pressed on a cylinder
    hardly any torsional friction, so a held cylinder swings about the
    closing axis, while flat faces hold firmly. The planner's grasps of
    round items close along face normals (EIGHT_YAWS, every other one of 16
    faces), but a perceived round item's yaw is arbitrary, so the pads may
    land on corners: with 16 faces the corners stick out 0.8 mm from the
    round visual of an 85 mm tape roll (an octagon's stuck out 3.5 mm, which
    showed as a gap between the fingers and the item)."""
    R = rpy_to_matrix(*rpy)
    a = d * math.tan(math.pi / sides)
    out = []
    lift = 0.0015 if base_disk else 0.0
    c = R @ np.array([0.0, 0.0, z + lift / 2])
    for k in range(sides // 2):
        rr = matrix_to_rpy(R @ rpy_to_matrix(0, 0, 2 * math.pi * k / sides))
        out.append(Shape("box", (d, a, h - lift), tuple(c), tuple(rr)))
    if base_disk:
        # The item stands on a thin disk; overlapping boxes on the table
        # would make DART solve many times as many contacts.
        out.append(Shape("cylinder", (d / 2, 2 * lift), tuple(R @ np.array([0.0, 0.0, z - h / 2 + lift])), rpy))
    return out


def round_footprint(d, sides=16):
    """Footprint of round_prism(d, ...) seen from above (its circumscribed circle)."""
    return circle(d / 2 / math.cos(math.pi / sides))


def ring_boxes(r_in, r_out, z0, z1, n=16, tilt_out=0.0):
    """Hollow cylinder (or frustum when tilt_out != 0) built from n boxes.

    The boxes overlap a little at the corners so there are no gaps.
    tilt_out: change of radius per unit height (frustum wall slope).
    """
    shapes = []
    h = z1 - z0
    t = r_out - r_in
    zc = (z0 + z1) / 2
    rc_mid = (r_in + r_out) / 2 + tilt_out * h / 2
    phi = math.atan(tilt_out)
    # Tangential length sized for the widest (top) end so neighbours overlap.
    seg = 2 * (r_out + tilt_out * h) * math.tan(math.pi / n) * 1.06
    length = h / math.cos(phi)
    for k in range(n):
        a = 2 * math.pi * k / n
        cx, cy = rc_mid * math.cos(a), rc_mid * math.sin(a)
        # Box x is radial, y tangential, z along the wall, which leans
        # outwards by phi (pitch about the box y axis, then yaw to angle a).
        shapes.append(Shape("box", (t, seg, length), (cx, cy, zc), (0.0, phi, a)))
    return shapes


def convex_hull(points):
    """Convex hull (counter-clockwise) of 2D points."""
    pts = sorted(map(tuple, np.asarray(points)))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])
