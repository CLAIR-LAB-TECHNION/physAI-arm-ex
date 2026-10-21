"""Where an item rests, from the 3D points of its segmentation mask.

The items are known (config/items.yaml, lab_replica/objects), so their
placement (support, resting pose, x, y, yaw: planning.placement.Placement)
is fitted to the points with the item's own dimensions:

    resting pose   the stable pose whose height matches the highest points
    x, y           the top surface seen from above: a circle of the known
                   radius for round items, a rectangle for blocks and for a
                   can lying on its side, the trigger head of the spray bottle
    yaw            the rectangle's direction; the mug's handle; the spray
                   bottle's nozzle (the side of the head away from the body)

Points are world points (z up, the table top at z = table_z). The camera is
assumed to look roughly down, so every item's top is visible.

The estimate itself is the exercise's (exercise/state_estimation.py
estimate_placement); this module has the fits it uses (circle_fit,
rect_fit, ...), the mug's and the spray bottle's shapes, the measuring of
a new item and the basket.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares

from ..planning.placement import Placement
from ..transforms import wrap


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


# ---- the item kinds (the simple ones are in exercise/state_estimation.py) ----------------


def estimate_item(item, pts, table_z, basket=None, basket_xy=None):
    """Estimate of `item` (an objects.* model) from world points (N, 3), or
    None when there are too few points: the exercise's estimate_placement
    (exercise/state_estimation.py)."""
    from ..exercise.state_estimation import estimate_placement
    return estimate_placement(item, pts, table_z, basket, basket_xy)


def top_layer(pts, top_abs, band):
    return pts[pts[:, 2] > top_abs - band]


def fit_mug(item, rest, pts, top_abs):
    s = item.spec
    c, _ = circle_fit(top_layer(pts, top_abs, 0.008)[:, :2], s["top_diameter"] / 2)
    r_body = s["top_diameter"] / 2
    d = pts[:, :2] - c
    rr = np.hypot(d[:, 0], d[:, 1])
    handle = d[(rr > r_body + 0.007) & (rr < r_body + s["handle_reach"] + 0.012) & (pts[:, 2] < top_abs - 0.006)]
    if len(handle) >= 15:
        m = handle.mean(axis=0)
        return c, math.atan2(m[1], m[0]), []
    return c, 0.0, ["handle not seen"]


def mug_handle_yaw(item, est, context):
    """Yaw of a mug (its x axis points at the handle) from the points around
    its body that belong to no other object, or None. `est` is the mug's
    Estimate (centre and top), `context` world points (N, 3)."""
    s = item.spec
    p = est.placement
    r_body = s["top_diameter"] / 2
    d = context[:, :2] - np.array([p.x, p.y])
    rr = np.hypot(d[:, 0], d[:, 1])
    sel = ((rr > r_body + 0.006) & (rr < r_body + s["handle_reach"] + 0.012)
           & (context[:, 2] > p.z_support + 0.012) & (context[:, 2] < est.top_z - 0.006))
    d = d[sel]
    if len(d) < 15:
        return None
    ang = np.arctan2(d[:, 1], d[:, 0])
    hist, edges = np.histogram(ang, bins=24, range=(-math.pi, math.pi))
    k = int(np.argmax(hist))
    peak = (edges[k] + edges[k + 1]) / 2
    near = ang[np.abs(np.angle(np.exp(1j * (ang - peak)))) < math.radians(40)]
    if len(near) < 15:
        return None
    return float(np.angle(np.exp(1j * near).mean()))


def fit_spray_bottle(item, rest, pts, top_abs):
    s = item.spec
    head = top_layer(pts, top_abs, s["head_height"] * 0.6)
    hc, _, a = rect_fit(head[:, :2])
    body = pts[pts[:, 2] < top_abs - s["head_height"] - s["neck_height"] - 0.01]
    u = np.array([math.cos(a), math.sin(a)])
    notes = []
    if len(body) >= 30:
        bc, _, _ = rect_fit(body[:, :2])   # the body's outline, seen from above
        # The nozzle side of the head sticks out beyond the body.
        if np.dot(hc - bc, u) < 0:
            a = wrap(a + math.pi)
    else:
        notes.append("body not seen: nozzle direction unknown")
    u = np.array([math.cos(a), math.sin(a)])
    return hc - s["head_offset"] * u, a, notes


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
