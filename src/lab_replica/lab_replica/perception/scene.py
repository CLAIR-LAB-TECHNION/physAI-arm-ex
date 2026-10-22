"""The scene from wrist-camera frames: which item is where, and the basket.

For every frame SAM 3 is asked for every item by name (config/perception.yaml
prompts). A name may match several objects and one object several names, so
each candidate mask is turned into 3D points, its placement is estimated
with the item's model (estimate.py) and scored by SAM's confidence times the
geometric plausibility; then masks are assigned to items greedily, best
first, one mask per item and one item per mask.

A scene grows one frame at a time (add_view). Which item is where comes from
any frame: per item its best-scoring detection (the exercise's fuse_views).
How it rests comes from the best frame for geometry that sees all of it:
looking straight down before looking at an angle, nearer before farther
(view_rank). A frame that sees an item cut off by the image's edge says
nothing about it, and one that finds an item more than SAME_PLACE from where
the scene has it is taken to have mistaken something else for it.
"""

import math
from dataclasses import dataclass, field, replace

import cv2
import numpy as np

from .. import config
from ..exercise import state_estimation as se
from .estimate import estimate_basket, estimate_item


def load_perception_config():
    import yaml
    with open(config.path("config", "perception.yaml")) as f:
        return yaml.safe_load(f)


@dataclass
class Detection:
    item: str
    score: float           # SAM score x plausibility
    sam_score: float
    estimate: object       # estimate.Estimate
    mask: np.ndarray
    view: int = 0
    frame: object = None    # the rgbd.Frame it was found in
    geometry_frame: object = None   # the frame its estimate comes from, when not `frame`


@dataclass
class Scene:
    detections: dict = field(default_factory=dict)    # item -> Detection
    basket_xy: np.ndarray = None
    frames: list = field(default_factory=list)
    per_view: list = field(default_factory=list)      # per frame: {item: Detection} kept from it

    def world(self, exclude=()):
        """{item: Placement} of the detected items, for the planner."""
        return {k: d.estimate.placement for k, d in self.detections.items() if k not in exclude}

    def missing(self, items):
        return [i for i in items if i not in self.detections]


class ScenePerception:
    def __init__(self, cell, items, basket, sam=None, cfg=None):
        self.cell = cell
        self.items = items
        self.basket = basket
        self.cfg = cfg or load_perception_config()
        self.table_z = cell["table"]["surface_z"]
        if sam is None:
            from .sam3 import Sam3
            sam = Sam3(self.cfg["sam3"]["model"])
        self.sam = sam
        self.prompt_item = {}
        for item, ps in self.cfg["prompts"].items():
            self.add_prompts(item, ps)

    def add_prompts(self, item, prompts):
        """Look for `item` (already in self.items) by these text prompts too."""
        for p in prompts:
            self.prompt_item[p] = item

    def remove_prompts(self, item):
        self.prompt_item = {p: it for p, it in self.prompt_item.items() if it != item}

    # ---- one frame ------------------------------------------------------------------

    def _points(self, frame, mask, erode_px=None):
        k = int(self.cfg["sam3"]["mask_erode_px"]) if erode_px is None else erode_px
        if k > 0:
            mask = cv2.erode(mask.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)) > 0
        P = frame.points_world(mask)
        keep = (P[:, 2] > self.table_z - 0.01) & (P[:, 2] < self.table_z + self.cfg["max_item_height"])
        keep &= np.hypot(P[:, 0], P[:, 1]) > self.cfg["robot_radius"]
        return P[keep]

    def _world(self, frame):
        """World points (H, W, 3) of every pixel of `frame` (NaN without
        depth); computed once per frame."""
        if getattr(frame, "world", None) is None:
            H, W = frame.depth.shape
            vv, uu = np.mgrid[0:H, 0:W]
            ok = np.isfinite(frame.depth) & (frame.depth > 0)
            P = np.full((H, W, 3), np.nan)
            P[ok] = se.pixels_to_world(uu[ok], vv[ok], frame.depth[ok], frame.K, frame.T_world_cam)
            frame.world = P
        return frame.world

    def _around(self, frame, centre, radius, blocked):
        """The world points within `radius` of `centre` (x, y), table
        included, except the pixels `blocked` and those near the robot."""
        P = self._world(frame)
        keep = ~blocked & (np.hypot(P[..., 0] - centre[0], P[..., 1] - centre[1]) < radius)
        P = P[keep]
        P = P[(P[:, 2] > self.table_z - 0.01) & (P[:, 2] < self.table_z + self.cfg["max_item_height"])]
        return P[np.hypot(P[:, 0], P[:, 1]) > self.cfg["robot_radius"]]

    def points_around(self, frame, mask, item, others=()):
        """The world points (N, 3) around the item `item` (a model) whose mask
        is `mask`: as far as it reaches from the mask's middle plus
        AROUND_MARGIN, the table there too, without the pixels of the masks
        `others` (grown by OTHERS_GROW_PX: their edges mix depths)."""
        pts = self._points(frame, mask, erode_px=0)
        if len(pts) == 0:
            return pts
        blocked = np.zeros(mask.shape, bool)
        for m in others:
            blocked |= _grow(m)
        blocked &= ~mask
        return self._around(frame, np.median(pts[:, :2], axis=0), item_reach(item) + AROUND_MARGIN, blocked)

    def detect(self, frame, items=None, basket_xy=None, view=0, exclude=()):
        """(detections {item: Detection}, basket_xy, basket mask) in one frame.

        items: the items to look for (default all but `exclude`); basket_xy:
        where the basket is when already known (else it is found here)."""
        items = [i for i in (items or self.items) if i not in exclude]
        prompts = [p for p, it in self.prompt_item.items() if it in items]
        bp = self.cfg["basket_prompt"]
        res = self.sam.segment(frame.rgb, prompts + [bp], threshold=self.cfg["sam3"]["threshold"])
        basket_mask = res[bp][0].mask if res[bp] else None
        if basket_xy is None and basket_mask is not None:
            basket_xy = estimate_basket(self.basket, self._points(frame, basket_mask), self.table_z)
        # Each item is fitted to the points around its mask, without those of
        # the other things: the masks SAM 3 is confident of (a weak one may
        # be a wrong name on part of this very item) and the basket. How many
        # of them (grown) cover each pixel, less the mask's own and those of
        # the same object under another name; the mask's own pixels always
        # count (the basket's mask covers the items in it).
        insts = [(self.prompt_item[p], inst) for p in prompts for inst in res[p]]
        sure = [k for k, (_, inst) in enumerate(insts) if inst.score >= CONFIDENT_SCORE]
        grown = {k: _grow(insts[k][1].mask) for k in sure}
        boxes = [_bbox(inst.mask) for _, inst in insts]
        covered = np.zeros(frame.depth.shape, np.int16)
        for g in grown.values():
            covered += g
        if basket_mask is not None:
            covered += _grow(basket_mask)
        cands = []
        for k, (item, inst) in enumerate(insts):
            pts = self._points(frame, inst.mask)
            if len(pts) < 30:
                continue
            mine = covered.copy()
            for j in sure:
                if j == k or (_boxes_meet(boxes[j], boxes[k]) and _overlap(insts[j][1].mask, inst.mask) > 0.5):
                    mine -= grown[j]
            around = self._around(frame, np.median(pts[:, :2], axis=0),
                                  item_reach(self.items[item]) + AROUND_MARGIN, (mine > 0) & ~inst.mask)
            est = estimate_item(self.items[item], pts, around, self.table_z, self.basket, basket_xy)
            if est is not None:
                cands.append(Detection(item, inst.score * est.plausibility, inst.score, est, inst.mask, view, frame))
        # One mask per item, one item per mask (the exercise's assign_masks).
        return se.assign_masks(cands, self.cfg["min_assignment_score"]), basket_xy

    # ---- several frames -------------------------------------------------------------

    def observe(self, frames, exclude=()):
        """Scene from several frames, added one after the other (add_view)."""
        scene = None
        for f in frames:
            scene = self.add_view(scene, f, exclude=exclude)
        return scene

    def add_view(self, scene, frame, exclude=(), log=None):
        """Add `frame` to `scene` (a new Scene when None) and return it: the
        items found in it join the scene's detections (fuse_views), and every
        item's estimate comes from the best frame for its geometry. `log`
        (f(text)) hears about the detections left out."""
        scene = scene if scene is not None else Scene()
        k = len(scene.frames)
        dets, bxy = self.detect(frame, basket_xy=scene.basket_xy, view=k, exclude=exclude)
        if scene.basket_xy is None and bxy is not None:
            scene.basket_xy = np.asarray(bxy, float)
        kept = {}
        for item, d in dets.items():
            p = d.estimate.placement
            old = scene.detections.get(item)
            if not whole_in_view(frame, self.items[item], p):
                why = "only partly in view"
            elif old is not None and math.hypot(old.estimate.placement.x - p.x,
                                                old.estimate.placement.y - p.y) > SAME_PLACE:
                why = (f"found {100 * math.hypot(old.estimate.placement.x - p.x, old.estimate.placement.y - p.y):.0f}"
                       f" cm from where it was seen before")
            else:
                kept[item] = d
                continue
            if log is not None:
                log(f"  view {k + 1}: {item} (score {d.score:.2f}) {why}: left out")
        scene.frames.append(frame)
        scene.per_view.append(kept)
        # Which item is where: every frame counts the same (the exercise's fuse_views).
        scene.detections = se.fuse_views(scene.per_view, [1.0] * len(scene.frames))
        self._geometry(scene)
        return scene

    def _geometry(self, scene):
        """Every detection's estimate from the best-ranked frame (view_rank)
        that sees all of the item: its detection there, else its depth there
        around where the item is (refit); when that fails, the next best."""
        order = sorted(range(len(scene.frames)), key=lambda k: view_rank(scene.frames[k], self.table_z))
        for item, d in list(scene.detections.items()):
            model = self.items[item]
            for k in order:
                f = scene.frames[k]
                if f is d.frame:
                    break
                if not whole_in_view(f, model, d.estimate.placement):
                    continue
                there = scene.per_view[k].get(item)
                if there is not None:
                    scene.detections[item] = replace(there, score=max(there.score, d.score))
                    break
                others = [o.mask for o in scene.per_view[k].values()]
                est = self.refit(item, d.estimate, f, others, scene.basket_xy)
                if est is not None:
                    est.notes.append(f"geometry from view {k + 1}")
                    scene.detections[item] = replace(d, estimate=est, geometry_frame=f)
                    break

    def refit(self, item, est, frame, other_masks=(), basket_xy=None, max_shift=0.04):
        """Re-estimate `item` from the depth of `frame` around where `est`
        put it, without the pixels of `other_masks`; None if that does not
        give a plausible estimate near the old one."""
        p = est.placement
        model = self.items[item]
        blocked = np.zeros(frame.depth.shape, bool)
        for m in other_masks:
            blocked |= _grow(m)
        fp = model.footprint_world(p.rest, p.x, p.y, p.yaw)
        c = fp.mean(axis=0)
        r = np.max(np.hypot(*(fp - c).T))
        around = self._around(frame, c, r + AROUND_MARGIN, blocked)
        P = around[(around[:, 2] > p.z_support + 0.004) & (np.hypot(around[:, 0] - c[0], around[:, 1] - c[1]) < r)]
        P = largest_cluster(P)
        new = estimate_item(model, P, around, self.table_z, self.basket, basket_xy)
        if new is None or new.plausibility < 0.5:
            return None
        q = new.placement
        if np.hypot(q.x - p.x, q.y - p.y) > max_shift:
            return None
        return new


AROUND_MARGIN = 0.025    # the points around an item reach this far beyond it [m]
SAME_PLACE = 0.05        # an item found again farther than this from where it was is something else [m]
TOP_DOWN_TILT = 10.0     # a view this close to straight down [deg] counts as looking straight down
EDGE_PX = 3              # an item counts as in view when its model stays this far inside the image
CONFIDENT_SCORE = 0.5    # SAM 3 masks this sure leave their points out of the other items' fits
OTHERS_GROW_PX = 3       # other masks are grown by this many pixels before their points are left out


def item_reach(item):
    """How far the item reaches from its middle seen from above, in any of
    its resting poses [m]."""
    return max(float(np.max(np.hypot(*rp.footprint.T))) for rp in item.poses.values())


def _grow(mask):
    k = 2 * OTHERS_GROW_PX + 1
    return cv2.dilate(mask.astype(np.uint8), np.ones((k, k), np.uint8)) > 0


def _bbox(mask):
    rows, cols = np.any(mask, axis=1), np.any(mask, axis=0)
    if not rows.any():
        return (0, -1, 0, -1)
    r, c = np.nonzero(rows)[0], np.nonzero(cols)[0]
    return (r[0], r[-1], c[0], c[-1])


def _boxes_meet(a, b):
    return a[0] <= b[1] and b[0] <= a[1] and a[2] <= b[3] and b[2] <= a[3]


def view_rank(frame, table_z):
    """Sort key of a frame for geometry, best first: looking straight down
    (within TOP_DOWN_TILT) before looking at an angle; straight down, the
    nearer to the table the better; at an angle, the less tilted the better."""
    T = frame.T_world_cam
    tilt = math.degrees(math.acos(float(np.clip(-T[2, 2], -1.0, 1.0))))
    if tilt <= TOP_DOWN_TILT:
        return (0, float(T[2, 3] - table_z))
    return (1, tilt)


def _model_points(model, placement):
    """World points outlining the item's collision shapes, resting as
    `placement`: a list of (N, 3) arrays, one per shape."""
    from ..planning.placement import placement_pose
    from ..transforms import rpy_to_matrix
    xyz, R = placement_pose(model, placement)
    out = []
    for sh in model.shapes:
        if sh.kind == "box":
            g = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float) / 2
            local = g * np.array(sh.size)
        else:
            r, h = sh.size
            a = np.linspace(0, 2 * math.pi, 32, endpoint=False)
            ring = np.c_[r * np.cos(a), r * np.sin(a)]
            local = np.vstack([np.c_[ring, np.full(len(a), z)] for z in (-h / 2, h / 2)])
        out.append((local @ rpy_to_matrix(*sh.rpy).T + np.array(sh.xyz)) @ R.T + xyz)
    return out


def whole_in_view(frame, model, placement, margin=EDGE_PX):
    """The item's model, resting as `placement`, lies inside `frame`'s image
    (`margin` pixels from its edges) in front of the camera."""
    H, W = frame.depth.shape
    uv, d = frame.project(np.vstack(_model_points(model, placement)))
    return bool(np.all(d > 0.01) and np.all((uv[:, 0] >= margin) & (uv[:, 0] < W - margin)
                                            & (uv[:, 1] >= margin) & (uv[:, 1] < H - margin)))


def model_mask(frame, model, placement):
    """The pixels of `frame` that the item's model, resting as `placement`,
    covers (its collision shapes' silhouettes; what hides it is ignored)."""
    H, W = frame.depth.shape
    mask = np.zeros((H, W), np.uint8)
    for P in _model_points(model, placement):
        uv, d = frame.project(P)
        if np.all(d > 0.01):
            cv2.fillConvexPoly(mask, cv2.convexHull(uv.astype(np.int32)), 1)
    return mask > 0


def height_image(frame):
    """World z of every pixel of `frame` (NaN without depth)."""
    from ..exercise.state_estimation import pixels_to_world
    H, W = frame.depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    ok = np.isfinite(frame.depth) & (frame.depth > 0)
    z = np.full((H, W), np.nan)
    z[ok] = pixels_to_world(uu[ok], vv[ok], frame.depth[ok], frame.K, frame.T_world_cam)[:, 2]
    return z


def largest_cluster(P, cell=0.006):
    """The points of the largest group connected in the horizontal plane
    (items stand at least 2 cm apart, so bits of neighbours fall away)."""
    if len(P) < 10:
        return P
    lo = P[:, :2].min(axis=0)
    ij = ((P[:, :2] - lo) / cell).astype(int)
    grid = np.zeros(ij.max(axis=0) + 1, np.uint8)
    grid[ij[:, 0], ij[:, 1]] = 1
    n, lab = cv2.connectedComponents(grid, connectivity=8)
    if n <= 2:
        return P
    labels = lab[ij[:, 0], ij[:, 1]]
    best = np.bincount(labels)[1:].argmax() + 1
    return P[labels == best]


def _overlap(a, b):
    """Intersection over the smaller mask."""
    from ..toolkit import mask_overlap
    return mask_overlap(a, b)


def draw(frame, detections, basket_xy=None, title=None):
    """BGR image of `frame` with the masks found in it, and the names and
    estimated centres of all detections."""
    vis = frame.rgb[:, :, ::-1].copy()
    rng = np.random.default_rng(7)
    for item, d in sorted(detections.items()):
        col = tuple(int(c) for c in rng.integers(70, 256, 3))
        if d.frame is frame:
            m = d.mask
            vis[m] = (0.45 * vis[m] + 0.55 * np.array(col)).astype(np.uint8)
            cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, cnts, -1, col, 2)
        p = d.estimate.placement
        uv, _ = frame.project(np.array([[p.x, p.y, d.estimate.top_z]]))
        u, v = uv[0]
        if 0 <= u < vis.shape[1] and 0 <= v < vis.shape[0]:
            fs = vis.shape[1] / 1800.0     # readable in a video inset
            cv2.drawMarker(vis, (int(u), int(v)), (255, 255, 255), cv2.MARKER_CROSS, 18, 3)
            cv2.putText(vis, f"{item} {d.score:.2f}", (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, fs,
                        (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(vis, f"{item} {d.score:.2f}", (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, fs,
                        (255, 255, 255), 2, cv2.LINE_AA)
    if basket_xy is not None:
        uv, _ = frame.project(np.array([[basket_xy[0], basket_xy[1], 0.12]]))
        cv2.drawMarker(vis, tuple(int(v) for v in uv[0]), (0, 200, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
    if title:
        cv2.putText(vis, title, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(vis, title, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return vis
