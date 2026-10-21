"""State estimation: from the wrist camera's images to where the items are,
and how to grasp one.

The executive (autonomy/executive.py) takes RGB-D frames from the wrist
camera and asks SAM 3 for every item by name. Each answer is a mask (a
boolean image) with a score. These functions turn them into the scene:

    pixels_to_world      depth pixels -> 3D points in the world frame
    estimate_placement   one item's points -> where and how it rests
    assign_masks         the candidate masks of one frame -> at most one per item
    fuse_views           the detections of several frames -> one per item
    grasp_in_image       an item's mask -> top-down grasps in the image
    grasp_pose           a grasp in the image -> the gripper's 3D pose and opening

Conventions (see lab_replica/toolkit.py for the types):
  * The world frame has z up, z = 0 on the table top (cell.yaml).
  * The camera's optical frame has z along the view, x to the right in the
    image and y down; a pixel (u, v) is column u, row v.
  * K is the 3x3 intrinsic matrix, T_world_cam the 4x4 pose of the optical
    frame in the world.
"""

import math

import cv2
import numpy as np

from ..toolkit import (Estimate, Grasp, ImageGrasp, Placement, circle_fit, fit_mug, fit_spray_bottle,
                       gauss, gripper_pose, mask_overlap, rect_fit, rg6, robust_top, top_layer, wrap)

# ---- from pixels to the world -------------------------------------------------------------


def pixels_to_world(u, v, depth, K, T_world_cam):
    """World points (N, 3) of the pixels (u[i], v[i]) (arrays of length N)
    whose depth along the camera's view (z of the optical frame, metres) is
    depth[i]."""
    u, v, z = (np.asarray(a, float) for a in (u, v, depth))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    P = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1)
    return P @ T_world_cam[:3, :3].T + T_world_cam[:3, 3]


# ---- one item: where it rests ---------------------------------------------------------------


def estimate_placement(item, pts, table_z, basket=None, basket_xy=None):
    """How `item` rests, from the world points (N, 3) of its mask: an
    Estimate (its Placement, how plausible the points are for this item,
    ...), or None when there are too few points.

    item       the item's model (objects/*.py): .kind, .spec (items.yaml),
               .poses {resting pose: RestPose (.top: its height, .footprint)}
    basket     the basket's model (objects/basket.py), basket_xy its centre
               (None: not known; then everything is on the table)

    The support is the basket when the points are inside its rim, else the
    table. The resting pose is the one whose height matches the top of the
    points. x, y and yaw come from the shape seen from above."""
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
    top_abs = robust_top(pts[:, 2])
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
    fit = FITS.get(item.kind, fit_generic)
    xy, yaw, notes = fit(item, rest, pts, top_abs)
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
    plaus = gauss(dh, 0.025) * gauss(max(0.0, w - ext_model * 1.15), 0.03)
    p = Placement(item.name, rest, float(xy[0]), float(xy[1]), float(yaw), support=support,
                  z_support=float(z_support))
    return Estimate(p, plaus, len(pts), top_abs, notes)


# The shape seen from above, per kind of item: fit(item, rest, pts, top_abs)
# -> ((x, y), yaw, notes). The mug's and the spray bottle's are given
# (toolkit.fit_mug, toolkit.fit_spray_bottle).

def fit_round(radius_key, band=0.008):
    """For round items: a circle of the known radius (item.spec[radius_key] / 2)
    through the top layer of the points; the yaw does not matter."""
    def fit(item, rest, pts, top_abs):
        r = item.spec[radius_key] / 2
        c, _ = circle_fit(top_layer(pts, top_abs, band)[:, :2], r)
        return c, 0.0, []
    return fit


def fit_block(item, rest, pts, top_abs):
    """A block lying flat: the rectangle of its top face; the yaw of a square
    is defined modulo 90 degrees."""
    c, (w, h), a = rect_fit(top_layer(pts, top_abs, 0.008)[:, :2])
    notes = [] if abs(w - item.spec["size"]) < 0.015 else [f"top face {w * 1000:.0f} x {h * 1000:.0f} mm"]
    return c, wrap(a % (math.pi / 2)), notes


def fit_can(item, rest, pts, top_abs):
    """Upright: round. Lying: the upper half of the cylinder is a rectangle
    along the can's axis, which lies along the rested frame's y axis."""
    if rest == "upright":
        return fit_round("diameter", 0.01)(item, rest, pts, top_abs)
    d = item.spec["diameter"]
    c, _, a = rect_fit(top_layer(pts, top_abs, d * 0.4)[:, :2])
    return c, wrap(a - math.pi / 2), []


def fit_measured(item, rest, pts, top_abs):
    """An item added at run time (objects/measured.py): round or a box."""
    if item.spec["shape"] == "round":
        c, _ = circle_fit(pts[:, :2], item.spec["size"][0] / 2)
        return c, 0.0, []
    h = item.spec["size"][2]
    c, _, a = rect_fit(top_layer(pts, top_abs, max(0.004, min(0.008, 0.4 * h)))[:, :2])
    return c, wrap(a % math.pi), []


def fit_generic(item, rest, pts, top_abs):
    c, _, a = rect_fit(pts[:, :2])
    return c, a, ["generic fit"]


FITS = {
    "block": fit_block,
    "tape_roll": fit_round("outer_diameter", 0.006),
    "cup": fit_round("top_diameter", 0.010),
    "can": fit_can,
    "mug": fit_mug,
    "spray_bottle": fit_spray_bottle,
    "measured": fit_measured,
}


# ---- which mask is which item -------------------------------------------------------------


def assign_masks(candidates, min_score, max_overlap=0.5):
    """At most one mask per item and one item per mask, from the candidates
    of one frame (a list of Detection: .item, .score, .mask, ...). A name may
    match several objects, and one object several names.

    Greedy, best score first: a candidate is taken unless its item already
    has a mask, its score is below min_score, or its mask overlaps a taken
    one by more than max_overlap (toolkit.mask_overlap). Returns {item: Detection}."""
    chosen = {}
    for d in sorted(candidates, key=lambda d: -d.score):
        if d.item in chosen or d.score < min_score:
            continue
        if any(mask_overlap(d.mask, o.mask) > max_overlap for o in chosen.values()):
            continue
        chosen[d.item] = d
    return chosen


def fuse_views(per_view, weights):
    """One detection per item from several frames: per_view[k] is the
    {item: Detection} of frame k, weights[k] how much that frame counts.

    Per item, the detection with the highest score x weight, unless another
    item already stands there (same support, centres within 2.5 cm, support
    heights within 3 cm: two items cannot be in one place); the items with
    the best detections choose first. Returns {item: Detection}."""
    ranked = {}
    for k, dets in enumerate(per_view):
        for item, d in dets.items():
            ranked.setdefault(item, []).append((d.score * weights[k], d))
    fused = {}
    order = sorted(((max(r, key=lambda t: t[0])[0], it) for it, r in ranked.items()), reverse=True)
    for _, item in order:
        for w, d in sorted(ranked[item], key=lambda t: -t[0]):
            p = d.estimate.placement
            clash = any(o.estimate.placement.support == p.support and
                        np.hypot(o.estimate.placement.x - p.x, o.estimate.placement.y - p.y) < 0.025 and
                        abs(o.estimate.placement.z_support - p.z_support) < 0.03
                        for o in fused.values())
            if not clash:
                fused[item] = d
                break
    return fused


# ---- grasping ---------------------------------------------------------------------------


GRASP_BAND = 0.012      # the top of an item: points this far below its highest
ROUND_ASPECT = 1.25     # a top this little longer than wide is not elongated
SQUARE_FILL = 0.83      # a top that fills this much of its rectangle is square (a disk fills pi / 4)


def grasp_in_image(frame, mask):
    """Top-down grasps of the item whose mask this is, in the image: each
    the pixel the fingers close around and the direction they close along.
    A list of ImageGrasp, best first ([] without enough depth in the mask).

    frame  the rgbd.Frame (.rgb, .depth in metres along the view, NaN where
           unknown, .K, .T_world_cam), taken looking straight down
    mask   the item's boolean mask in that frame

    The fingers close on the top of the item: the pixels of the mask that
    are nearest to the camera, around their centre.
      * An elongated top (a can lying down, the spray bottle's head): across
        its narrow side.
      * A square top (a block): across one pair of sides, then the other.
      * A round top (a cup, a can standing, a tape roll, the mug's rim): any
        direction; first across the narrow side of the whole mask, which
        keeps the fingers off the mug's handle, then turned in steps of
        22.5 degrees (only up to 45 degrees when the whole mask is
        elongated: the mug's handle)."""
    d = np.where(mask & np.isfinite(frame.depth) & (frame.depth > 0), frame.depth, np.nan)
    if np.count_nonzero(np.isfinite(d)) < 30:
        return []
    near = np.nanpercentile(d, 3)
    top = np.isfinite(d) & (d < near + GRASP_BAND)
    vv, uu = np.nonzero(top)
    pts = np.stack([uu, vv], axis=1).astype(np.float32)
    (cu, cv), (w, h), ang = cv2.minAreaRect(pts)
    # minAreaRect: `ang` (degrees) is the direction of the side of length w.
    across = math.radians(ang) + (math.pi / 2 if w > h else 0.0)     # across the shorter side
    narrow = float(min(w, h))
    if max(w, h) >= ROUND_ASPECT * min(w, h):
        angles = [across]
    elif cv2.contourArea(cv2.convexHull(pts)) >= SQUARE_FILL * w * h:
        angles = [across, across + math.pi / 2]
    else:
        va, ua = np.nonzero(mask)
        _, (W, H), A = cv2.minAreaRect(np.stack([ua, va], axis=1).astype(np.float32))
        across = math.radians(A) + (math.pi / 2 if W > H else 0.0)
        steps = [0, 1, -1, 2, -2] if max(W, H) >= 1.15 * min(W, H) else [0, 1, -1, 2, -2, 3, -3, 4]
        angles = [across + k * math.pi / 8 for k in steps]
    return [ImageGrasp(u=float(cu), v=float(cv), angle=wrap(a), width_px=narrow) for a in angles]


TIP_CLEARANCE = 0.008   # the fingertips stay this far above the table
KNUCKLE_MARGIN = 0.004  # the item's top stays this far below the knuckles


def grasp_pose(frame, image_grasp, mask, table_z):
    """The 3D grasp for a grasp in the image: a Grasp with the pose of the
    gripper's mounting face (rg6_base_link) at the grasp, the item's width
    between the pads and the fingertips' height.

    The centre pixel and a pixel on each side along the closing direction go
    back into the world at the depth of the item's top; the closing
    direction is their horizontal direction. The fingertips go as deep as
    the knuckles allow (the item's top must stay below them:
    rg6.free_height(width)), but not lower than TIP_CLEARANCE above the
    table. The width is how far the item's points between the fingertips
    and its top extend along the closing direction. The gripper points
    straight down, its fingers closing along its y axis."""
    g = image_grasp
    d = np.where(mask & np.isfinite(frame.depth) & (frame.depth > 0), frame.depth, np.nan)
    near = np.nanpercentile(d, 3)
    top = np.isfinite(d) & (d < near + GRASP_BAND)
    z_top_depth = float(np.nanmedian(d[top]))
    du, dv = math.cos(g.angle), math.sin(g.angle)
    r = max(g.width_px / 2, 5.0)
    P = pixels_to_world([g.u, g.u - r * du, g.u + r * du], [g.v, g.v - r * dv, g.v + r * dv],
                        [z_top_depth] * 3, frame.K, frame.T_world_cam)
    centre = P[0]
    axis = P[2, :2] - P[1, :2]
    closing = math.atan2(axis[1], axis[0])
    n = np.array([math.cos(closing), math.sin(closing)])
    # The item's 3D points, and how far they extend along the closing direction.
    vv, uu = np.nonzero(np.isfinite(d))
    Q = pixels_to_world(uu, vv, d[vv, uu], frame.K, frame.T_world_cam)
    z_top = float(np.percentile(Q[:, 2], 97))

    def extent(z_min):
        t = (Q[Q[:, 2] >= z_min, :2] - centre[:2]) @ n
        lo, hi = np.percentile(t, [0.5, 99.5])      # a stray pixel or two do not count
        return float(hi - lo), float(hi + lo) / 2

    # First the width of the top, then the width between the fingertips'
    # height and the top (the pads close there: a can lying down is wider
    # below its top).
    width, _ = extent(z_top - GRASP_BAND)
    tips = max(table_z + TIP_CLEARANCE, z_top - rg6.free_height(width) + KNUCKLE_MARGIN)
    width, mid = extent(tips)
    tips = max(table_z + TIP_CLEARANCE, z_top - rg6.free_height(width) + KNUCKLE_MARGIN)
    centre[:2] += n * mid
    yaw = wrap(closing - math.pi / 2)          # the gripper's x axis; the fingers close along its y axis
    pose = gripper_pose(centre[0], centre[1], tips + rg6.depth_from_width(width), yaw)
    return Grasp(pose=pose, width=width, tips_z=float(tips), top_z=z_top)
