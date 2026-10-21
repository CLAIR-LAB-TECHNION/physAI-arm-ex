"""Annotated camera views of what the robot decided, for /lab_skills/detections.

Every image gets a title banner (big enough to read in a video inset). The
put actions show their plan drawn into the robot's survey view: for the
basket, the rim and the pile heights seen in it and the drop spot; for the
table, what the camera saw (and what it could not see) that the spot keeps
clear of, and the chosen spot.

Each drawing takes a `frame` (perception.rgbd.Frame) to draw on; with
DepthLook(frame) instead it draws the same into the frame's depth image (the
console's camera window shows both, viewer.py).
"""

import math

import cv2
import numpy as np

GREEN = (80, 230, 80)
ORANGE = (0, 170, 255)
RED = (60, 60, 235)
WHITE = (255, 255, 255)


def banner(vis, text, sub=None):
    """A dark title strip across the top of `vis` (BGR), text scaled to fit."""
    W = vis.shape[1]
    h = int(round(W * 0.055)) + (int(round(W * 0.03)) if sub else 0)
    vis[:h] = (0.25 * vis[:h]).astype(np.uint8)
    scale = W / 800.0
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 3)
    scale = min(scale, scale * (W - 30) / max(tw, 1))
    y = int(round(W * 0.042))
    cv2.putText(vis, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, scale, WHITE, max(2, int(scale * 2)), cv2.LINE_AA)
    if sub:
        s2 = scale * 0.6
        cv2.putText(vis, sub, (14, y + int(round(W * 0.03))), cv2.FONT_HERSHEY_SIMPLEX, s2, (200, 200, 200),
                    max(1, int(s2 * 2)), cv2.LINE_AA)
    return vis


def label(vis, text, uv, color=WHITE, scale=None):
    scale = scale or vis.shape[1] / 1100.0
    t = max(1, int(round(scale * 2)))
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, t + 3)
    # Kept inside the image.
    u = int(np.clip(uv[0], 4, vis.shape[1] - tw - 4))
    v = int(np.clip(uv[1], th + 4, vis.shape[0] - 4))
    cv2.putText(vis, text, (u, v), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), t + 3, cv2.LINE_AA)
    cv2.putText(vis, text, (u, v), cv2.FONT_HERSHEY_SIMPLEX, scale, color, t, cv2.LINE_AA)


def _px(frame, P):
    """Pixels of world points P (N, 3); None if any is behind the camera."""
    uv, d = frame.project(np.asarray(P, float))
    if np.any(d <= 0.01):
        return None
    return np.round(uv).astype(np.int32)


def polyline(vis, frame, P, color, thick=2, closed=True):
    uv = _px(frame, P)
    if uv is not None:
        cv2.polylines(vis, [uv.reshape(-1, 1, 2)], closed, color, thick, cv2.LINE_AA)
    return uv


def circle3d(c, r, z, n=72):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(n, z)]


def prism(vis, frame, footprint, z0, z1, color, thick=2, fill=None):
    """Wire frame (and optionally a translucent bottom) of a vertical prism."""
    fp = np.asarray(footprint, float)
    lo = np.c_[fp, np.full(len(fp), z0)]
    hi = np.c_[fp, np.full(len(fp), z1)]
    if fill is not None:
        uv = _px(frame, lo)
        if uv is not None:
            ov = vis.copy()
            cv2.fillPoly(ov, [cv2.convexHull(uv)], fill, cv2.LINE_AA)
            cv2.addWeighted(ov, 0.35, vis, 0.65, 0, dst=vis)
    polyline(vis, frame, lo, color, thick)
    polyline(vis, frame, hi, color, thick)
    for k in range(0, len(fp), max(1, len(fp) // 8)):
        polyline(vis, frame, [lo[k], hi[k]], color, thick, closed=False)


def arrow(vis, frame, a, b, color, thick=3):
    uv = _px(frame, [a, b])
    if uv is not None:
        cv2.arrowedLine(vis, tuple(map(int, uv[0])), tuple(map(int, uv[1])), color, thick, cv2.LINE_AA,
                        tipLength=0.25)


class Zoomed:
    """A camera frame cropped to the pixels of the world points P (plus
    `margin` of their extent on each side, at the image's aspect ratio) and
    scaled back to full size, so the part that matters is readable in a
    small inset. Has the frame's rgb and project() for drawing."""

    def __init__(self, frame, P, margin=0.2):
        self.frame = frame
        H, W = frame.rgb.shape[:2]
        uv, d = frame.project(np.asarray(P, float))
        uv = uv[d > 0.01]
        if len(uv) == 0:
            uv = np.array([[0.0, 0.0], [W, H]])
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        c, size = (lo + hi) / 2, (hi - lo) * (1 + 2 * margin)
        w = min(max(size[0], size[1] * W / H, W * 0.3), W)
        h = w * H / W
        self.x0 = float(np.clip(c[0] - w / 2, 0, W - w))
        self.y0 = float(np.clip(c[1] - h / 2, 0, H - h))
        self.s = W / w
        M = np.array([[self.s, 0, -self.x0 * self.s], [0, self.s, -self.y0 * self.s]])
        self.rgb = cv2.warpAffine(frame.rgb, M, (W, H), flags=cv2.INTER_LINEAR)

    def project(self, P):
        uv, d = self.frame.project(P)
        return (uv - (self.x0, self.y0)) * self.s, d


class DepthLook:
    """A frame whose `rgb` is its depth image in colours (viewer.depth_colors),
    so the drawings here draw into the depth instead of the colour image."""

    def __init__(self, frame):
        from ..viewer import depth_colors
        self.frame = frame
        bgr, self.lo, self.hi = depth_colors(frame.depth)
        self.rgb = bgr[:, :, ::-1].copy()
        self.depth = frame.depth

    def project(self, P):
        return self.frame.project(P)


def source_frame(view):
    """The rgbd.Frame behind a frame, a DepthLook or a Zoomed view."""
    while hasattr(view, "frame"):
        view = view.frame
    return view


def item_color(item):
    """A fixed bright BGR colour per item name."""
    import zlib
    rng = np.random.default_rng(zlib.crc32(item.encode()))
    c = rng.integers(70, 256, 3)
    c[rng.integers(0, 3)] = 255
    return tuple(int(v) for v in c)


def _dim(frame):
    vis = frame.rgb[:, :, ::-1].copy()
    return (0.75 * vis).astype(np.uint8)


def draw_cells(vis, frame, depth, centre, half):
    """The occupied cells of a depth_world.DepthWorld within `half` of
    `centre`, drawn at their tops: seen red, unseen (bounded by the rays) purple."""
    from .depth_world import CELL as C
    X, Y = depth.centres()
    near = (np.abs(X - centre[0]) < half) & (np.abs(Y - centre[1]) < half) & depth.occupied
    ov = vis.copy()
    for j, i in zip(*np.nonzero(near)):
        xa, ya, zt = X[j, i] - C / 2, Y[j, i] - C / 2, float(depth.H[j, i])
        uv = _px(frame, [[xa, ya, zt], [xa + C, ya, zt], [xa + C, ya + C, zt], [xa, ya + C, zt]])
        if uv is not None:
            cv2.fillConvexPoly(ov, uv, (200, 60, 170) if depth.carved[j, i] else RED, cv2.LINE_AA)
    cv2.addWeighted(ov, 0.5, vis, 0.5, 0, dst=vis)


def draw_drop(frame, basket, basket_xy, table_z, drops, footprint, target, z_release, item, title):
    """The basket put: rim, pile heights seen inside (lighter magenta = higher),
    the item's footprint where it is released and where it should land."""
    frame = Zoomed(frame, circle3d(basket_xy, basket.r_top, table_z + basket.height), margin=0.12)
    vis = _dim(frame)
    H, x0, y0 = drops.heights, drops.x0, drops.y0
    from .obstacles import CELL
    ov = vis.copy()
    ny, nx = H.shape
    top = max(0.05, float(np.nanmax(H)) if np.isfinite(H).any() else 0.05)
    for j in range(ny):
        for i in range(nx):
            h = H[j, i]
            if not np.isfinite(h):
                continue
            xa, ya = x0 + i * CELL, y0 + j * CELL
            z = drops.z_floor + h
            uv = _px(frame, [[xa, ya, z], [xa + CELL, ya, z], [xa + CELL, ya + CELL, z], [xa, ya + CELL, z]])
            if uv is None:
                continue
            t = min(1.0, h / top)
            col = (int(150 + 105 * t), int(40 + 80 * t), int(150 + 105 * t)) if h > 0.01 else (90, 170, 60)
            cv2.fillConvexPoly(ov, uv, col, cv2.LINE_AA)
    cv2.addWeighted(ov, 0.45, vis, 0.55, 0, dst=vis)
    rim = table_z + basket.height
    polyline(vis, frame, circle3d(basket_xy, basket.r_top, rim), ORANGE, 3)
    polyline(vis, frame, circle3d(basket_xy, basket.inner_radius(0.0), drops.z_floor), ORANGE, 1)
    c = np.asarray(footprint, float).mean(axis=0)
    prism(vis, frame, footprint, z_release, z_release + 0.002, WHITE, 2)
    prism(vis, frame, footprint, target.z_support, target.z_support + 0.002, GREEN, 3, fill=GREEN)
    arrow(vis, frame, [c[0], c[1], z_release], [c[0], c[1], target.z_support + 0.005], GREEN, 3)
    uv = _px(frame, [[c[0], c[1], z_release]])
    if uv is not None:
        pile = target.z_support - drops.z_floor
        where = "on the floor" if pile < 0.01 else f"on a pile {pile * 100:.0f} cm high"
        label(vis, "drop here", uv[0] + [60, -14], GREEN)
        label(vis, f"lands {where}", uv[0] + [60, 30], GREEN)
    banner(vis, title, "rim orange; depth inside: green free, magenta piled")
    return vis


def draw_table(frame, cell, items, basket, scene, region, footprint, target, source, item, title):
    """The table put: the free region, what is kept clear of (everything the
    camera saw above the table, red; space it could not see, purple; the
    basket), and the chosen spot."""
    pts = [[target.x + dx, target.y + dy, cell["table"]["surface_z"]] for dx in (-0.15, 0.15) for dy in (-0.15, 0.15)]
    if source is not None:
        pts.append([source.x, source.y, cell["table"]["surface_z"]])
    frame = Zoomed(frame, pts, margin=0.1)
    vis = _dim(frame)
    z = cell["table"]["surface_z"]
    (xa, xb), (ya, yb) = region["x_range"], region["y_range"]
    polyline(vis, frame, [[xa, ya, z], [xb, ya, z], [xb, yb, z], [xa, yb, z]], WHITE, 1)
    for cx, cy, r in region.get("keepout", []):
        polyline(vis, frame, circle3d((cx, cy), r, z), ORANGE, 2)
    depth = getattr(scene, "depth", None)
    if depth is not None:
        draw_cells(vis, frame, depth, [target.x, target.y], 0.25)
    top = items[item].poses[target.rest].top
    prism(vis, frame, footprint, z, z + top, GREEN, 3, fill=GREEN)
    if source is not None and math.hypot(source.x - target.x, source.y - target.y) > 0.01:
        uv = _px(frame, [[source.x, source.y, z]])
        if uv is not None:
            cv2.drawMarker(vis, tuple(map(int, uv[0])), WHITE, cv2.MARKER_TILTED_CROSS, 22, 3)
            label(vis, "picked up here", uv[0] + [20, 40])
    uv = _px(frame, [[target.x, target.y, z + top]])
    if uv is not None:
        label(vis, "put here", uv[0] + [60, -14], GREEN)
    banner(vis, title, "kept clear of: seen red, unseen purple, basket and robot orange")
    return vis


# ---- every observation, the grasp, a new item ---------------------------------------------------


def draw_perception(frame, scene, items, basket, table_z, title, sub=None):
    """What the robot made of one view: per item found in it its SAM 3 mask,
    its model where it is estimated (wire frame) and its name and score
    (SAM 3 x plausibility); the basket's rim; the unknown obstacles (red)
    and the space no view saw (purple), when the depth was mapped."""
    src = source_frame(frame)
    vis = frame.rgb[:, :, ::-1].copy()
    for item, d in sorted(scene.detections.items()):
        col = item_color(item)
        if d.frame is src:
            m = d.mask
            vis[m] = (0.5 * vis[m] + 0.5 * np.array(col)).astype(np.uint8)
            cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, cnts, -1, col, 2)
    for o in getattr(scene, "obstacles", []) or []:
        P = np.c_[o.polygon, np.full(len(o.polygon), o.z_top)]
        polyline(vis, frame, P, (200, 60, 170) if o.unseen else RED, 2)
    if scene.basket_xy is not None:
        polyline(vis, frame, circle3d(scene.basket_xy, basket.r_top, table_z + basket.height), ORANGE, 2)
    for item, d in sorted(scene.detections.items()):
        p = d.estimate.placement
        model = items.get(item)
        if model is None:
            continue
        col = item_color(item)
        top = model.poses[p.rest].top
        prism(vis, frame, model.footprint_world(p.rest, p.x, p.y, p.yaw), p.z_support, p.z_support + top, col, 1)
        uv = _px(frame, [[p.x, p.y, p.z_support + top]])
        if uv is not None and 0 <= uv[0][0] < vis.shape[1] and 0 <= uv[0][1] < vis.shape[0]:
            cv2.drawMarker(vis, tuple(map(int, uv[0])), WHITE, cv2.MARKER_CROSS, 14, 2)
            label(vis, f"{item} {d.score:.2f}", uv[0] + [8, -8], col, scale=vis.shape[1] / 1500.0)
    banner(vis, title, sub or "masks; wire frames: estimated items; orange: basket; obstacles: red seen, "
                              "purple unseen")
    return vis


def gripper_boxes(gx, gy, psi, width):
    """Footprints of the two RG6 pads (finger ends) at `width` apart; the
    closing direction is psi + 90 deg (planning/skills.py finger_boxes)."""
    from ..robot import rg6
    d = np.array([math.cos(psi + math.pi / 2), math.sin(psi + math.pi / 2)])
    t = np.array([-d[1], d[0]])
    out = []
    for side in (-1, 1):
        c = np.array([gx, gy]) + side * d * (width / 2 + rg6.FINGER_THICKNESS / 2)
        hw, hd = rg6.FINGER_HALF_WIDTH, rg6.FINGER_THICKNESS / 2
        out.append(np.array([c + a * hd * d + b * hw * t for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]))
    return out, d


def draw_grasp(frame, model, placement, held, open_width, force, title):
    """The planned grasp: the item's model where it is estimated, the pads
    open (white) and where they close on it (green), and the closing
    direction."""
    from ..robot import rg6
    p = placement
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    ox, oy = held.offset_obj
    gx, gy = p.x + c * ox - s * oy, p.y + s * ox + c * oy
    psi = p.yaw - held.yaw_rel
    z0 = p.z_support + held.tip_z
    top = model.poses[p.rest].top
    fp = model.footprint_world(p.rest, p.x, p.y, p.yaw)
    pts = np.r_[np.c_[fp, np.full(len(fp), p.z_support)], [[gx, gy, z0 + 0.12]]]
    frame = Zoomed(frame, pts, margin=0.6)
    vis = _dim(frame)
    prism(vis, frame, fp, p.z_support, p.z_support + top, item_color(p.item), 2)
    opened, d = gripper_boxes(gx, gy, psi, open_width)
    closed, _ = gripper_boxes(gx, gy, psi, held.width)
    for b in opened:
        prism(vis, frame, b, z0, z0 + rg6.PAD_LENGTH, WHITE, 1)
    for b in closed:
        prism(vis, frame, b, z0, z0 + rg6.PAD_LENGTH, GREEN, 2, fill=GREEN)
    for side in (-1, 1):
        a = np.array([gx, gy]) + side * d * (open_width / 2 + 0.012)
        b = np.array([gx, gy]) + side * d * (held.width / 2 + 0.004)
        arrow(vis, frame, [a[0], a[1], z0 + 0.025], [b[0], b[1], z0 + 0.025], GREEN, 2)
    uv = _px(frame, [[gx, gy, z0 + rg6.PAD_LENGTH]])
    if uv is not None:
        label(vis, f"grasp across {held.width * 1000:.0f} mm, {force:.0f} N", uv[0] + [40, -30], GREEN)
        label(vis, f"fingertips {held.tip_z * 1000:.0f} mm above the {p.support}", uv[0] + [40, 10], GREEN)
    banner(vis, title, "white: fingers open on the way down; green: where they close")
    return vis


def draw_image_grasp(frame, mask, grasps, title):
    """Grasps proposed in the image (exercise: state_estimation.grasp_in_image),
    best first: the item's mask, the pixel the fingers close around, the
    closing direction and the item's width across it (the two pads); the
    first grasp bold, the others thin."""
    vis = _dim(frame)
    if mask is not None and mask.shape == vis.shape[:2]:
        vis[mask] = (0.5 * vis[mask] + 0.5 * np.array(ORANGE)).astype(np.uint8)
    for k, grasp in reversed(list(enumerate(grasps))):
        c = np.array([grasp.u, grasp.v])
        d = np.array([math.cos(grasp.angle), math.sin(grasp.angle)])
        t = np.array([-d[1], d[0]])
        half = grasp.width_px / 2
        pad = max(6.0, 0.35 * grasp.width_px)
        col, th = (GREEN, 3) if k == 0 else (WHITE, 1)
        for side in (-1, 1):
            a = c + side * d * (half + 0.3 * grasp.width_px + 12)
            b = c + side * d * half
            cv2.arrowedLine(vis, tuple(int(v) for v in a), tuple(int(v) for v in b), col, th, cv2.LINE_AA,
                            tipLength=0.3)
            p0, p1 = b + side * d * 4 + t * pad, b + side * d * 4 - t * pad
            cv2.line(vis, tuple(int(v) for v in p0), tuple(int(v) for v in p1), col, th + 1, cv2.LINE_AA)
    if grasps:
        g = grasps[0]
        cv2.drawMarker(vis, (int(g.u), int(g.v)), WHITE, cv2.MARKER_CROSS, 16, 2)
        label(vis, f"({g.u:.0f}, {g.v:.0f}) px, {math.degrees(g.angle):.0f} deg, {g.width_px:.0f} px across",
              (8, vis.shape[0] - 10), GREEN)
    banner(vis, title, "orange: the item's pixels; green: the first grasp (pads and closing direction), "
                       "white: the others")
    return vis


def draw_measure(frame, mask, model, placement, what, title, sub=None):
    """A new item: its SAM 3 mask and the model made from the measurement."""
    vis = frame.rgb[:, :, ::-1].copy()
    if mask is not None and mask.shape == vis.shape[:2]:
        vis[mask] = (0.5 * vis[mask] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)
    if model is not None and placement is not None:
        p = placement
        top = model.poses[p.rest].top
        prism(vis, frame, model.footprint_world(p.rest, p.x, p.y, p.yaw), p.z_support, p.z_support + top, GREEN, 2)
        uv = _px(frame, [[p.x, p.y, p.z_support + top]])
        if uv is not None:
            label(vis, what, uv[0] + [30, -20], GREEN, scale=vis.shape[1] / 1300.0)
    banner(vis, title, sub)
    return vis
