"""The scene from wrist-camera frames: which item is where, and the basket.

For every frame SAM 3 is asked for every item by name (config/perception.yaml
prompts). A name may match several objects and one object several names, so
each candidate mask is turned into 3D points, its placement is estimated
with the item's model (estimate.py) and scored by SAM's confidence times the
geometric plausibility; then masks are assigned to items greedily, best
first, one mask per item and one item per mask. Several frames are merged by
keeping, per item, its best-scoring estimate.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

from .. import config
from .estimate import estimate_basket, estimate_item, mug_handle_yaw


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


@dataclass
class Scene:
    detections: dict = field(default_factory=dict)    # item -> Detection
    basket_xy: np.ndarray = None
    frames: list = field(default_factory=list)

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
            for p in ps:
                self.prompt_item[p] = item

    # ---- one frame ------------------------------------------------------------------

    def _points(self, frame, mask):
        k = int(self.cfg["sam3"]["mask_erode_px"])
        if k > 0:
            mask = cv2.erode(mask.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)) > 0
        P = frame.points_world(mask)
        keep = (P[:, 2] > self.table_z - 0.01) & (P[:, 2] < self.table_z + self.cfg["max_item_height"])
        keep &= np.hypot(P[:, 0], P[:, 1]) > self.cfg["robot_radius"]
        return P[keep]

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
        cands = []
        for p in prompts:
            item = self.prompt_item[p]
            for inst in res[p]:
                pts = self._points(frame, inst.mask)
                est = estimate_item(self.items[item], pts, self.table_z, self.basket, basket_xy)
                if est is not None:
                    cands.append(Detection(item, inst.score * est.plausibility, inst.score, est, inst.mask, view,
                                           frame))
        # A mask may cover only part of its item (the spray bottle's body
        # without its grey head) and then fit badly: fit it again to all the
        # depth points around it, except those of masks that already fit
        # another item well.
        confident = [d for d in cands if d.score >= 0.5]
        for d in cands:
            if d.estimate.plausibility >= 0.5:
                continue
            others = [o.mask for o in confident if o.item != d.item and _overlap(o.mask, d.mask) < 0.3]
            if basket_mask is not None:
                others.append(basket_mask)
            est2 = self.refit(d.item, d.estimate, frame, others, basket_xy=basket_xy, grow=0.03, max_shift=0.06)
            if est2 is not None and est2.plausibility > d.estimate.plausibility:
                est2.notes.append("grown beyond the mask")
                d.estimate, d.score = est2, d.sam_score * est2.plausibility
        cands.sort(key=lambda d: -d.score)
        chosen = {}
        for d in cands:
            if d.item in chosen or d.score < self.cfg["min_assignment_score"]:
                continue
            if any(_overlap(d.mask, o.mask) > 0.5 for o in chosen.values()):
                continue
            chosen[d.item] = d
        # The mug's handle is usually not part of its mask: look for it in
        # the depth points around the body that belong to nothing else.
        for item, d in chosen.items():
            if self.items[item].kind != "mug":
                continue
            others = np.zeros(frame.depth.shape, bool)
            for o in chosen.values():
                if o is not d:
                    others |= o.mask
            if basket_mask is not None:
                others |= basket_mask
            ctx = self._points(frame, ~others)
            yaw = mug_handle_yaw(self.items[item], d.estimate, ctx)
            if yaw is not None:
                d.estimate.placement.yaw = yaw
                d.estimate.notes = [n for n in d.estimate.notes if n != "handle not seen"]
        return chosen, basket_xy

    # ---- several frames -------------------------------------------------------------

    def observe(self, frames, weights=None, items=None, basket_xy=None, exclude=()):
        """Scene from several frames: per item the detection with the highest
        score x weight of its frame."""
        weights = list(weights or [1.0] * len(frames))
        scene = Scene(frames=list(frames))
        best_basket = None
        ranked = {}
        for k, f in enumerate(frames):
            dets, bxy = self.detect(f, items, basket_xy, view=k, exclude=exclude)
            if bxy is not None and (best_basket is None or weights[k] > best_basket[0]):
                best_basket = (weights[k], np.asarray(bxy, float))
            for item, d in dets.items():
                ranked.setdefault(item, []).append((d.score * weights[k], d))
        scene.basket_xy = np.asarray(basket_xy, float) if basket_xy is not None else (
            best_basket[1] if best_basket else None)
        # Best detection per item; two items cannot stand in one place.
        order = sorted(((max(r, key=lambda t: t[0])[0], it) for it, r in ranked.items()), reverse=True)
        for _, item in order:
            for w, d in sorted(ranked[item], key=lambda t: -t[0]):
                p = d.estimate.placement
                clash = any(o.estimate.placement.support == p.support and
                            np.hypot(o.estimate.placement.x - p.x, o.estimate.placement.y - p.y) < 0.025 and
                            abs(o.estimate.placement.z_support - p.z_support) < 0.03
                            for o in scene.detections.values())
                if not clash:
                    scene.detections[item] = d
                    break
        # Items recognised only in a lower-weight (oblique) view: take their
        # geometry from the best view's depth, around where they were found.
        k_best = int(np.argmax(weights))
        best = frames[k_best]
        for item, d in list(scene.detections.items()):
            if weights[d.view] >= weights[k_best]:
                continue
            others = [o.mask for o in scene.detections.values() if o.frame is best]
            est = self.refit(item, d.estimate, best, others, scene.basket_xy)
            if est is not None:
                est.notes.append(f"geometry from view {k_best + 1}")
                d.estimate = est
        return scene

    def refit(self, item, est, frame, other_masks=(), basket_xy=None, grow=0.025, max_shift=0.04):
        """Re-estimate `item` from the depth of `frame` inside its footprint
        (grown by `grow`), without the pixels of `other_masks`; None if that
        does not give a plausible estimate near the old one."""
        p = est.placement
        model = self.items[item]
        H, W = frame.depth.shape
        keep = np.ones((H, W), bool)
        for m in other_masks:
            keep &= ~m
        P = frame.points_world(keep, stride=2)
        P = P[(P[:, 2] > p.z_support + 0.004) & (P[:, 2] < p.z_support + model.poses[p.rest].top + 0.03)]
        fp = model.footprint_world(p.rest, p.x, p.y, p.yaw)
        c = fp.mean(axis=0)
        rad = np.max(np.hypot(*(fp - c).T)) + grow
        P = P[np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < rad]
        P = P[np.hypot(P[:, 0], P[:, 1]) > self.cfg["robot_radius"]]
        P = largest_cluster(P)
        new = estimate_item(model, P, self.table_z, self.basket, basket_xy)
        if new is None or new.plausibility < 0.5:
            return None
        q = new.placement
        if np.hypot(q.x - p.x, q.y - p.y) > max_shift:
            return None
        return new


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
    inter = np.logical_and(a, b).sum()
    return inter / max(1, min(a.sum(), b.sum()))


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
