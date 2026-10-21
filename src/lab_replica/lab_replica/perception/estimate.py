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


def _robust_top(z):
    return float(np.percentile(z, 97)) if len(z) else float("nan")


def _gauss(err, scale):
    return math.exp(-(err / scale) ** 2)


# ---- the item kinds --------------------------------------------------------------------


def estimate_item(item, pts, table_z, basket=None, basket_xy=None):
    """Estimate of `item` (an objects.* model) from world points (N, 3), or
    None when there are too few points."""
    pts = np.asarray(pts, float)
    if len(pts) < 30:
        return None
    support, z_floor = "table", table_z
    if basket is not None and basket_xy is not None:
        c = np.median(pts[:, :2], axis=0)
        if math.hypot(*(c - basket_xy)) < basket.inner_radius(basket.height) - 0.005:
            support, z_floor = "basket", table_z + basket.floor
    pts = pts[pts[:, 2] > z_floor + 0.004]
    if len(pts) < 30:
        return None
    top_abs = _robust_top(pts[:, 2])
    # Resting pose: the one whose height matches. In the basket the item may
    # lie on others, so it is only matched against the floor if that fits.
    rests = list(item.spec["stable_poses"])
    h = top_abs - z_floor
    rest = min(rests, key=lambda r: abs(item.poses[r].top - h))
    rp = item.poses[rest]
    if support == "basket":
        z_support = max(z_floor, top_abs - rp.top)
    else:
        z_support = table_z
    fn = _KINDS.get(item.kind, _generic)
    xy, yaw, notes = fn(item, rest, pts, top_abs)
    # Plausibility: height of the top above the support and the size of the
    # outline must match the model.
    if support == "table":
        dh = abs(rp.top - (top_abs - z_support))
    else:
        # In the basket an item may rest on others (taller is fine), but it
        # cannot be shorter than its model.
        dh = max(0.0, rp.top - (top_abs - z_floor))
    fp = rp.footprint
    ext_model = max(np.ptp(fp[:, 0]), np.ptp(fp[:, 1]))
    _, (w, _), _ = rect_fit(pts[:, :2])
    plaus = _gauss(dh, 0.025) * _gauss(max(0.0, w - ext_model * 1.15), 0.03)
    p = Placement(item.name, rest, float(xy[0]), float(xy[1]), float(yaw), support=support,
                  z_support=float(z_support))
    return Estimate(p, plaus, len(pts), top_abs, notes)


def _top_layer(pts, top_abs, band):
    return pts[pts[:, 2] > top_abs - band]


def _round(radius_key, band=0.008):
    def f(item, rest, pts, top_abs):
        r = item.spec[radius_key] / 2
        c, _ = circle_fit(_top_layer(pts, top_abs, band)[:, :2], r)
        return c, 0.0, []
    return f


def _block(item, rest, pts, top_abs):
    c, (w, h), a = rect_fit(_top_layer(pts, top_abs, 0.008)[:, :2])
    notes = [] if abs(w - item.spec["size"]) < 0.015 else [f"top face {w * 1000:.0f} x {h * 1000:.0f} mm"]
    return c, wrap(a % (math.pi / 2)), notes


def _can(item, rest, pts, top_abs):
    if rest == "upright":
        return _round("diameter", 0.01)(item, rest, pts, top_abs)
    # Lying: the upper half of the cylinder is a rectangle along the can's
    # axis, which lies along the rested frame's y axis.
    d = item.spec["diameter"]
    c, _, a = rect_fit(_top_layer(pts, top_abs, d * 0.4)[:, :2])
    return c, wrap(a - math.pi / 2), []


def _mug(item, rest, pts, top_abs):
    s = item.spec
    c, _ = circle_fit(_top_layer(pts, top_abs, 0.008)[:, :2], s["top_diameter"] / 2)
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


def _spray_bottle(item, rest, pts, top_abs):
    s = item.spec
    head = _top_layer(pts, top_abs, s["head_height"] * 0.6)
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


def _measured(item, rest, pts, top_abs):
    """An item added at run time (objects/measured.py)."""
    if item.spec["shape"] == "round":
        c, _ = circle_fit(pts[:, :2], item.spec["size"][0] / 2)
        return c, 0.0, []
    h = item.spec["size"][2]
    c, _, a = rect_fit(_top_layer(pts, top_abs, max(0.004, min(0.008, 0.4 * h)))[:, :2])
    return c, wrap(a % math.pi), []


def _generic(item, rest, pts, top_abs):
    c, _, a = rect_fit(pts[:, :2])
    return c, a, ["generic fit"]


_KINDS = {
    "block": _block,
    "tape_roll": _round("outer_diameter", 0.006),
    "cup": _round("top_diameter", 0.010),
    "can": _can,
    "mug": _mug,
    "spray_bottle": _spray_bottle,
    "measured": _measured,
}


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
    h = _robust_top(pts[:, 2]) - table_z
    top = _top_layer(pts, table_z + h, max(0.004, min(0.01, 0.4 * h)))
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
