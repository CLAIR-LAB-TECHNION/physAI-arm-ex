"""Where an item rests, from the 3D points of its segmentation mask.

The items are known (config/items.yaml, lab_replica/objects), so their
placement (support, resting pose, x, y, yaw: planning.placement.Placement)
is fitted with the item's own model: the exercise's estimate_placement
(exercise/state_estimation.py) matches the model's top surface seen from
above (model_height_map, from its collision shapes) to the heights of the
points around the item, for every item kind alike.

Points are world points (z up, the table top at z = table_z). The camera is
assumed to look roughly down, so every item's top is visible.

This module has what the estimate uses (model_height_map, rect_fit, ...),
the measuring of a new item and the basket.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares

from ..planning.placement import Placement
from ..transforms import rpy_to_matrix, wrap


@dataclass
class Estimate:
    placement: Placement
    plausibility: float          # 0..1: how well the points match the item's size
    n_points: int
    top_z: float                 # highest point (robust) above the world origin
    notes: list = field(default_factory=list)


# ---- small geometric fits ------------------------------------------------------------


def circle_fit(xy, r=None):
    """Centre (and radius) of a circle through 2D points; with r given, only
    the centre is fitted. Uses the convex hull, i.e. the outline seen from
    above, so a filled disk or a ring both give their outer circle."""
    xy = np.asarray(xy, float)
    if len(xy) < 5:
        return xy.mean(axis=0), r or 0.0
    hull = cv2.convexHull(xy.astype(np.float32)).reshape(-1, 2).astype(float)
    if len(hull) < 5:
        hull = xy
    c0 = xy.mean(axis=0)
    if r is None:
        res = least_squares(lambda p: np.hypot(*(hull - p[:2]).T) - p[2], [c0[0], c0[1], np.ptp(hull[:, 0]) / 2])
        return res.x[:2], abs(res.x[2])
    res = least_squares(lambda c: np.hypot(*(hull - c).T) - r, c0, loss="soft_l1", f_scale=0.003)
    return res.x, r


def rect_fit(xy):
    """(centre, (long side, short side), direction of the long side) of the
    minimum-area rectangle around 2D points."""
    (cx, cy), (w, h), ang = cv2.minAreaRect(np.asarray(xy, np.float32))
    a = math.radians(ang)
    if w < h:
        w, h = h, w
        a += math.pi / 2
    return np.array([cx, cy]), (w, h), wrap(a)


def robust_top(z):
    return float(np.percentile(z, 97)) if len(z) else float("nan")


def gauss(err, scale):
    return math.exp(-(err / scale) ** 2)


# ---- the item's model seen from above -------------------------------------------------


def estimate_item(item, pts, around, table_z, basket=None, basket_xy=None):
    """Estimate of `item` (an objects.* model) from the world points (N, 3)
    of its mask and those around it, or None when there are too few points:
    the exercise's estimate_placement (exercise/state_estimation.py)."""
    from ..exercise.state_estimation import estimate_placement
    return estimate_placement(item, pts, around, table_z, basket, basket_xy)


def top_layer(pts, top_abs, band):
    return pts[pts[:, 2] > top_abs - band]


class ModelHeightMap:
    """An item's top surface seen from above, resting in one of its poses:
    the height above the support of its highest point at each cell of a grid
    in the rested frame (x, y as in RestPose.footprint), 0 where the item is
    not. Made from the item's collision shapes (objects/*.py), so every kind
    of item has one, items added at run time too.

        hm = model_height_map(item, rest)
        hm.at(u, v)    heights at the points (u[i], v[i]) of the rested frame
    """

    def __init__(self, item, rest, cell):
        rp = item.poses[rest]
        R0 = rpy_to_matrix(*rp.rpy)
        self.cell = cell
        self.origin = rp.footprint.min(axis=0) - 3 * cell
        size = rp.footprint.max(axis=0) + 3 * cell - self.origin
        X, Y = np.meshgrid(self.origin[0] + cell * np.arange(int(size[0] / cell) + 1),
                           self.origin[1] + cell * np.arange(int(size[1] / cell) + 1))
        self.heights = np.zeros(X.shape)
        for sh in item.shapes:
            # The highest point of the shape on the vertical line through each
            # cell: the line in the shape's frame, clipped to its box (a
            # cylinder counts as its bounding box: only the thin base disks
            # and the spray bottle's neck are cylinders).
            R = R0 @ rpy_to_matrix(*sh.rpy)
            c = R0 @ np.asarray(sh.xyz, float) + np.array([0.0, 0.0, rp.z])
            half = (np.array(sh.size) / 2 if sh.kind == "box"
                    else np.array([sh.size[0], sh.size[0], sh.size[1] / 2]))
            o = np.stack([X - c[0], Y - c[1], np.full(X.shape, -c[2])], axis=-1) @ R
            d = R[2]                                   # the vertical, in the shape's frame
            t_in, t_out = np.full(X.shape, -np.inf), np.full(X.shape, np.inf)
            for k in range(3):
                if abs(d[k]) < 1e-9:
                    t_out[np.abs(o[..., k]) > half[k]] = -np.inf
                    continue
                a, b = (-half[k] - o[..., k]) / d[k], (half[k] - o[..., k]) / d[k]
                t_in, t_out = np.maximum(t_in, np.minimum(a, b)), np.minimum(t_out, np.maximum(a, b))
            hit = t_in <= t_out
            self.heights[hit] = np.maximum(self.heights[hit], t_out[hit])
        self.heights = self.heights.astype(np.float32)

    def at(self, u, v):
        # The grid has a border of empty cells: points off it read 0 there.
        ny, nx = self.heights.shape
        i = np.clip(((np.asarray(v) - self.origin[1]) / self.cell + 0.5).astype(np.int32), 0, ny - 1)
        j = np.clip(((np.asarray(u) - self.origin[0]) / self.cell + 0.5).astype(np.int32), 0, nx - 1)
        return self.heights.take(i * nx + j)


_HEIGHT_MAPS = {}


def model_height_map(item, rest, cell=0.002):
    """The ModelHeightMap of `item` resting as `rest` (made once per item,
    resting pose and cell size)."""
    key = (item.name, id(item), rest, cell)
    if key not in _HEIGHT_MAPS:
        _HEIGHT_MAPS[key] = ModelHeightMap(item, rest, cell)
    return _HEIGHT_MAPS[key]


# ---- measuring a new item ----------------------------------------------------------------

ROUND_FILL = 0.83   # outline area / its minimum rectangle's: 1 for a box, pi / 4 for a disk;
                    # low, because a box taken for round would be grasped across its corners


def measure_item(pts, table_z, shape=None):
    """Size of an unknown item standing on the table, from the world points
    of its mask: (shape, size, notes) with shape 'box' (size [length, width,
    height], length >= width) or 'round' (size [diameter, height]); None with
    too few points. `shape` forces the shape (default: from the outline of
    its top, a disk fills pi / 4 of its square, a rectangle all of it).

    The points are 3D, so the side walls a slanted view sees fall inside the
    outline; percentiles drop the odd stray pixel at the mask's edge."""
    pts = np.asarray(pts, float)
    pts = pts[pts[:, 2] > table_z + 0.004]
    if len(pts) < 30:
        return None
    h = robust_top(pts[:, 2]) - table_z
    top = top_layer(pts, table_z + h, max(0.004, min(0.01, 0.4 * h)))
    (cx, cy), (w0, h0), ang = cv2.minAreaRect(top[:, :2].astype(np.float32))
    notes = []
    if shape is None:
        hull = cv2.convexHull(top[:, :2].astype(np.float32))
        fill = cv2.contourArea(hull) / max(w0 * h0, 1e-9)
        shape = "round" if fill < ROUND_FILL else "box"
        notes.append(f"top outline fills {fill:.2f} of its rectangle: {shape}")
    a = math.radians(ang)
    u = np.array([math.cos(a), math.sin(a)])
    v = np.array([-u[1], u[0]])
    xy = pts[:, :2] - np.array([cx, cy])
    ext = []
    for d in (u, v):
        t = xy @ d
        lo, hi = np.percentile(t, [1, 99])
        ext.append(float(hi - lo))
    length, width = max(ext), min(ext)
    if shape == "round":
        return "round", [round((length + width) / 2, 4), round(h, 4)], notes
    return "box", [round(length, 4), round(width, 4), round(h, 4)], notes


# ---- the basket ------------------------------------------------------------------------


def estimate_basket(basket, pts, table_z):
    """Centre (x, y) of the basket from the points of its mask: a circle of
    the rim's radius through the rim. None if the rim is not seen."""
    pts = np.asarray(pts, float)
    rim_z = table_z + basket.height
    rim = pts[(pts[:, 2] > rim_z - 0.025) & (pts[:, 2] < rim_z + 0.015)]
    if len(rim) < 50:
        return None
    c, _ = circle_fit(rim[:, :2], basket.r_top - basket.wall / 2)
    return c
