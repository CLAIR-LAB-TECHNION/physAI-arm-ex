"""The planner on perceived scenes, and helpers for the depth seen by the camera.

PerceivedSkills is the pick-and-place planner (planning/skills.py) with the
obstacle model replaced: instead of the item models at their fitted poses it
keeps clear of what the wrist camera measured (depth_world.DepthWorld). The
helpers here remove the robot's own pixels from the depth and make coarse
height maps (dropping into the basket, pictures).
"""

from dataclasses import dataclass

import numpy as np

from ..planning.skills import Skills
from ..planning.placement import placement_pose
from ..robot.collision import Box, object_box

CELL = 0.01           # resolution of height_map() [m]
NEAR_BASE = 0.35      # m from the base axis
PLACED_PAD = 0.02     # an item put down during this action (not seen there yet): its box grown by this


@dataclass
class Obstacle:
    polygon: np.ndarray    # (N, 2) convex outline
    z_top: float
    support: str           # 'table' or 'basket'
    unseen: bool = False   # mostly cells the camera did not see (bounded by its rays)


_ROBOT = {}


class RobotSurface:
    """The robot's collision geometry sampled over its surface (about one
    point per `spacing`), for removing the robot's own pixels from the depth.
    Link frames come from planning's RobotPoints (same URDF)."""

    def __init__(self, cell, spacing=0.01):
        import xml.etree.ElementTree as ET
        from ..robot.collision import RobotPoints, _read_stl, resolve_uri
        from ..robot.kinematics import robot_urdf
        from ..transforms import rpy_to_matrix
        urdf = robot_urdf(cell["robot"]["ur_type"])
        self.rp = RobotPoints(urdf)
        rng = np.random.default_rng(0)
        self.points = {}
        for link in ET.fromstring(urdf).findall("link"):
            pts = []
            for c in link.findall("collision"):
                g = c.find("geometry")[0]
                o = c.find("origin")
                xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
                if g.tag == "mesh":
                    tri = _read_stl(resolve_uri(g.get("filename"))).reshape(-1, 3, 3)
                    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
                    n = max(50, int(area.sum() / spacing ** 2))
                    k = rng.choice(len(tri), n, p=area / area.sum())
                    a, b = rng.uniform(size=(2, n, 1))
                    swap = (a + b) > 1
                    a, b = np.where(swap, 1 - a, a), np.where(swap, 1 - b, b)
                    v = tri[k, 0] + a * (tri[k, 1] - tri[k, 0]) + b * (tri[k, 2] - tri[k, 0])
                elif g.tag == "box":
                    size = np.array([float(v) for v in g.get("size").split()])
                    n = max(30, int(2 * (size[0] * size[1] + size[1] * size[2] + size[0] * size[2]) / spacing ** 2))
                    v = rng.uniform(-0.5, 0.5, (n, 3)) * size
                    ax = rng.integers(0, 3, n)
                    v[np.arange(n), ax] = np.sign(v[np.arange(n), ax]) * size[ax] / 2
                elif g.tag == "cylinder":
                    r, h = float(g.get("radius")), float(g.get("length"))
                    n = max(30, int((2 * np.pi * r * h + 2 * np.pi * r * r) / spacing ** 2))
                    t = rng.uniform(0, 2 * np.pi, n)
                    v = np.c_[r * np.cos(t), r * np.sin(t), rng.uniform(-h / 2, h / 2, n)]
                else:
                    continue
                pts.append(v @ rpy_to_matrix(*rpy).T + np.array(xyz))
            if pts:
                self.points[link.get("name")] = np.vstack(pts)

    def world_points(self, q, width=0.08):
        T = self.rp.frames(q, width)
        return np.vstack([P @ T[k][:3, :3].T + T[k][:3, 3] for k, P in self.points.items() if k in T])


def remove_robot(P, q, cell, width=0.08, dist=0.03):
    """Points farther than `dist` from the robot at joints q."""
    from scipy.spatial import cKDTree
    if "surface" not in _ROBOT:
        _ROBOT["surface"] = RobotSurface(cell)
    R = _ROBOT["surface"].world_points(q, width)
    d, _ = cKDTree(R).query(P, distance_upper_bound=dist)
    return P[~np.isfinite(d)]


def height_map(frames, cell, x_range=None, y_range=None, robot_radius=0.17, stride=3):
    """(heights (ny, nx), x0, y0): highest point per CELL, NaN where unseen.
    The area defaults to the placement region (cell.yaml) plus 5 cm."""
    pl = cell["placement"]
    x0, x1 = x_range or (pl["x_range"][0] - 0.05, pl["x_range"][1] + 0.05)
    y0, y1 = y_range or (pl["y_range"][0] - 0.05, pl["y_range"][1] + 0.05)
    t = cell["table"]
    nx, ny = int(np.ceil((x1 - x0) / CELL)), int(np.ceil((y1 - y0) / CELL))
    H = np.full((ny, nx), np.nan)
    for f in frames:
        P = f.points_world(stride=stride)
        P = P[(P[:, 2] > t["surface_z"] - 0.02) & (P[:, 2] < t["surface_z"] + 0.35)]
        r = np.hypot(P[:, 0], P[:, 1])
        # Near the base, anything higher than the tallest item (0.21 m) is
        # the robot's shoulder and upper arm.
        P = P[(r > robot_radius) & ((r > NEAR_BASE) | (P[:, 2] < t["surface_z"] + 0.24))]
        if f.q is not None:
            P = remove_robot(P, f.q, cell)
        ix = ((P[:, 0] - x0) / CELL).astype(int)
        iy = ((P[:, 1] - y0) / CELL).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        flat = np.full(nx * ny, -np.inf)
        np.maximum.at(flat, iy[ok] * nx + ix[ok], P[ok, 2])
        flat = np.maximum(np.nan_to_num(H, nan=-np.inf).ravel(), flat)
        H = np.where(np.isfinite(flat), flat, np.nan).reshape(ny, nx)
    return H, x0, y0


class PerceivedSkills(Skills):
    """The pick-and-place planner with what the camera saw as the obstacles.

    `world` ({item: Placement}) still says which items are where (grasps,
    placements) and which of the seen items count as obstacles: an item
    left out of `world` (the one picked up, the one in the gripper) has its
    cells left out of the depth. With no depth yet (before the first look)
    only the table, the basket and the robot itself are obstacles."""

    def __init__(self, cell, items, basket, basket_xy=None, depth=None):
        super().__init__(cell, items, basket, basket_xy=basket_xy)
        self.depth = depth

    def _unseen_items(self, world, exclude=()):
        """Items in `world` that the depth does not contain (put down during this action)."""
        return [k for k in world if k not in exclude and (self.depth is None or not self.depth.has(k))]

    def boxes(self, world, exclude=()):
        out = [] if self.depth is None else self.depth.layers([k for k in world if k not in exclude])
        for k in self._unseen_items(world, exclude):
            if self.depth is None:
                continue
            xyz, R = placement_pose(self.items[k], world[k])
            b = object_box(self.items[k], xyz, R, f"{k} (just put down)")
            out.append(Box(b.name, b.c, b.R, b.h + PLACED_PAD))
        return out

    def top_under(self, poly, world, support=None, margin=0.0):
        if self.depth is None:
            return -np.inf
        top = self.depth.top_under(poly, margin, list(world))
        placed = {k: world[k] for k in self._unseen_items(world)}
        if placed:
            top = max(top, super().top_under(poly, placed, None, margin + PLACED_PAD))
        return top if top > -100 else -np.inf
