"""Execute PDDL actions of the order-assembly domain on demand, closed loop.

    (pick-up X)        look (survey view; also from the front if X is not
                       recognised from above), find X, look at it from
                       closer, plan a collision-free top-down grasp, grasp,
                       lift, return home. Applicable when the gripper is empty.
    (put-in-basket X)  look, find the basket and how high things are piled in
                       it, carry X above its emptiest spot and drop it there
                       (it may land on other items), return home. Applicable
                       when the gripper holds something.
    (put-on-table X)   look, put X down on the free table spot nearest to
                       where it was picked up (or to target_xy), return home.

"Empty" and "holding" come from the gripper (the RG6's grip-detected flag);
nothing else of the PDDL state is assumed. Everything about the scene comes
from the wrist camera: item poses (perception/: where to grasp, which item
is which), the basket, and the depth seen (depth_world.py), which is what the
motions keep clear of: the item models are not obstacles. The motions are the
collision-checked pick-and-place plans of planning/skills.py, made from the
perceived scene instead of a known layout.

The same code runs in simulation and on the real robot; it talks to the arm
and gripper through execution.executor.Executor and to the camera through
the RealSense topics.
"""

import math
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from std_msgs.msg import Bool, Float64, String

from .. import config, objects
from ..perception.rgbd import RGBDCamera
from ..perception.scene import ScenePerception, load_perception_config
from ..planning.motion import PlanError
from ..planning.placement import Placement
from ..planning.skills import Held
from ..robot import rg6
from ..transforms import inv_tf, wrap
from . import annotate
from .depth_world import DepthWorld
from .drop import drop_spots, spread
from .obstacles import PerceivedSkills
from .views import ViewPlanner, down_view, view_from_config

ACTIONS = ("pick-up", "put-in-basket", "put-on-table")
HELD_VIEW_CLEAR = 0.03   # a view raises a held item this far above max_item_height (views.above_held)


class ActionError(RuntimeError):
    pass


@dataclass
class HeldItem:
    item: str
    held: Held              # how it is held (planning.skills.Held)
    source: Placement = None  # where it was picked up
    assumed: bool = False   # not seen being picked: nominal grasp assumed
    depth_before: object = None  # DepthWorld of the survey before the pick-up (not saved)


def parse_action(text):
    """'(pick-up green-mug)' -> ('pick-up', 'green-mug')."""
    parts = text.strip().strip("()").split()
    if len(parts) != 2 or parts[0] not in ACTIONS:
        raise ActionError(f"not an action of the domain: {text!r} (expected '(<{'|'.join(ACTIONS)}> <item>)')")
    return parts[0], parts[1]


class ActionExecutive:
    """`node` is an execution.executor.Executor (arm and gripper clients); it
    is spun from here with spin_once."""

    def __init__(self, node, log=print, perception=None, run_dir=None, return_home=True, save_frames=False):
        self.node = node
        self.log = log
        self.cell = config.load_cell()
        self.items = objects.load_items()
        self.basket = objects.load_basket(self.cell)
        self.cfg = load_perception_config()
        ur_type = self.cell["robot"]["ur_type"]
        self.cam = RGBDCamera(node, ur_type, self.cfg["topics"])
        self.perception = perception or ScenePerception(self.cell, self.items, self.basket, cfg=self.cfg)
        self.views = ViewPlanner(PerceivedSkills(self.cell, self.items, self.basket), ur_type)
        self.grip = {"detected": None, "width": None}
        node.create_subscription(Bool, "/rg6/grip_detected", lambda m: self.grip.__setitem__("detected", m.data), 10)
        node.create_subscription(Float64, "/rg6/width", lambda m: self.grip.__setitem__("width", m.data), 10)
        self.status_pub = node.create_publisher(String, "/lab_skills/status", 10)
        from sensor_msgs.msg import Image
        self.det_pub = node.create_publisher(Image, "/lab_skills/detections", 1)
        self.held = None
        self.held_file = os.path.join(os.path.expanduser("~"), ".ros", "lab_skills_held.json")
        self._load_held()
        self.scene = None
        self.return_home = return_home
        self.run_dir = run_dir
        self.save_frames = save_frames
        self.on_phase = None
        self.on_observation = None   # f(colour BGR, depth BGR, title) for each annotated view
        self.action = ""
        self.home = np.array(self.cell["robot"]["home_joints"], float)

    # ---- what is in the gripper (survives restarts) ---------------------------------------

    def _save_held(self):
        import json
        from dataclasses import asdict
        os.makedirs(os.path.dirname(self.held_file), exist_ok=True)
        data = None if self.held is None else {
            "item": self.held.item, "held": asdict(self.held.held), "assumed": self.held.assumed,
            "source": self.held.source.as_dict() if self.held.source else None}
        with open(self.held_file, "w") as f:
            json.dump(data, f)

    def _load_held(self):
        import json
        try:
            with open(self.held_file) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        if d and d["item"] not in self.items:
            # An item added in an earlier console session (new-item:) is not
            # known any more; the gripper's own flag still says it is not empty.
            self.log(f"remembered holding {d['item']}, which is not an item now: forgotten")
            return
        if d:
            h = d["held"]
            h["offset_obj"] = tuple(h["offset_obj"])
            self.held = HeldItem(d["item"], Held(**h), Placement(**d["source"]) if d["source"] else None,
                                 d.get("assumed", False))
            self.log(f"remembered: holding {self.held.item} (checked against the gripper at the next action)")

    def set_held(self, held):
        self.held = held
        self._save_held()

    # ---- small helpers ------------------------------------------------------------------

    def phase(self, text):
        self.log(f"  [{text}]")
        self.status_pub.publish(String(data=text))
        if self.on_phase:
            self.on_phase(text)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def holding(self):
        """Grip detected by the gripper (after a fresh reading)."""
        self.grip["detected"] = None
        t0 = time.time()
        # 15 s: on a network of several computers, a new node may hear the
        # gripper driver only ~5 s after it starts (discovery).
        while self.grip["detected"] is None and time.time() - t0 < 15.0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.grip["detected"] is None:
            raise ActionError("no reading of /rg6/grip_detected: is the gripper driver running?")
        return bool(self.grip["detected"])

    def width(self):
        w = self.grip["width"]
        return float(w) if w is not None and math.isfinite(w) else rg6.width_from_q(-0.3)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def run(self, steps):
        self.node.run_steps(steps, log=lambda s: self.log(s))

    def traj(self, label, path):
        return self.views.sk._traj(label, path)

    # ---- looking --------------------------------------------------------------------------

    def capture(self, settle=0.4):
        """A frame taken with the arm at rest, with the camera pose."""
        self.spin(0.1)
        t = self.now() + settle
        frame = self.cam.capture(after=t)
        q = self.node.current()
        frame.q = q
        frame.T_world_cam = self.cam.pose(q)
        if self.save_frames and self.run_dir:
            os.makedirs(self.run_dir, exist_ok=True)
            np.savez_compressed(os.path.join(self.run_dir, f"frame_{time.strftime('%H%M%S')}_{frame.stamp:.1f}.npz"),
                                rgb=frame.rgb, depth=frame.depth, K=frame.K, T=frame.T_world_cam, q=q)
        return frame

    def look(self, T_cam, world, held, label, depth=None):
        """Move the camera to T_cam and take a frame. The motion keeps clear
        of what the camera saw last (`depth`, default the last scene's;
        `world` says which of its items are still there), unless that blocks
        every way: it may have changed since."""
        q = self.node.current()
        if depth is None:
            depth = getattr(self.scene, "depth", None)
        if held is not None:
            z_clear = self.cell["table"]["surface_z"] + self.cfg["max_item_height"] + HELD_VIEW_CLEAR
            T_cam = self.views.above_held(T_cam, held, q, z_clear)
        try:
            self.views.sk.depth = depth
            path = self.views.move_to_view(q, T_cam, world, self.width(), held=held, label=label)
        except PlanError as e:
            if depth is None:
                raise
            self.log(f"  {e}; looking without what was seen before")
            self.views.sk.depth = None
            path = self.views.move_to_view(q, T_cam, world, self.width(), held=held, label=label)
        finally:
            self.views.sk.depth = None
        self.run([self.traj(label, path)])
        return self.capture()

    def survey(self, exclude=(), held=None, target=None):
        """Look from the survey views (and, when `target` is not recognised
        well enough from them, also from the recognition views); returns the
        scene (perception.scene.Scene) with .depth (depth_world.DepthWorld)
        and .obstacles (the unrecognised things in it) set."""
        world = self.scene.world(exclude=exclude) if self.scene else {}
        frames, weights = [], []

        def take(views):
            for v in views:
                self.phase(f"observing: {v.get('name', 'view')} view")
                frames.append(self.look(view_from_config(v), world, held, f"look ({v.get('name', 'view')})"))
                weights.append(float(v.get("weight", 1.0)))
            self.phase("recognising the items (SAM 3)")
            return self.perception.observe(frames, weights, exclude=exclude)

        t0 = time.time()
        scene = take(self.cfg["survey_views"])
        extra = self.cfg.get("recognition_views", [])
        if target is not None and extra:
            d = scene.detections.get(target)
            if d is None or d.score < float(self.cfg.get("recognition_min_score", 0.5)):
                self.log(f"  {target} {'not recognised' if d is None else f'recognised with score {d.score:.2f}'}"
                         f" from above; looking from the side too")
                t0 = time.time()
                scene = take(extra)
        if scene.basket_xy is None:
            raise ActionError("the basket is not in view")
        self.phase("mapping what the camera saw")
        self.map_depth(scene, frames, held)
        found = sorted(scene.detections)
        self.log(f"  found {len(found)} items in {time.time() - t0:.1f} s: {', '.join(found)}"
                 + (f"; not found: {', '.join(scene.missing([i for i in self.items if i not in exclude]))}"
                    if len(found) < len(self.items) - len(exclude) else "")
                 + (f"; {len(scene.obstacles)} unknown obstacle(s)" if scene.obstacles else ""))
        for o in scene.obstacles:
            c = o.polygon.mean(axis=0)
            self.log(f"    {'unseen space' if o.unseen else 'unknown obstacle'} at ({c[0]:.3f}, {c[1]:.3f}), "
                     f"{np.ptp(o.polygon[:, 0]) * 100:.0f} x {np.ptp(o.polygon[:, 1]) * 100:.0f} cm, "
                     f"{o.z_top * 100:.0f} cm high, on the {o.support}")
        # Every view, annotated (a put then also shows its plan).
        views = self.cfg["survey_views"] + extra
        for k, f in enumerate(frames):
            self.publish_detections(f, scene, f"{views[k].get('name', 'survey')} view: SAM 3 + depth")
        self.save_scene(scene, "survey")
        self.scene = scene
        return scene

    def close_look(self, scene, item, held=None):
        """Look at `item` from closer and update the estimates of the items
        seen there (when they agree with the survey within 5 cm). Returns
        the frame, or None without a closer look."""
        cl = self.cfg.get("close_look", {})
        if not cl.get("enabled", False):
            return None
        p = scene.detections[item].estimate.placement
        frame = self.look_down_at(p.x, p.y, cl["height"], scene.world(), held)
        if frame is None:
            self.log(f"  no close look at {item} possible; using the survey")
            return None
        self.phase(f"recognising {item} from closer")
        sub = self.perception.observe([frame], [float(cl.get("weight", 1.2))], basket_xy=scene.basket_xy)
        for it, d in sub.detections.items():
            old = scene.detections.get(it)
            if old is None:
                continue
            a, b = old.estimate.placement, d.estimate.placement
            if math.hypot(a.x - b.x, a.y - b.y) < 0.05:
                if it == item:
                    self.log(f"  {item}: closer look moved it by {1000 * math.hypot(a.x - b.x, a.y - b.y):.0f} mm")
                scene.detections[it] = d
        self.map_depth(scene, scene.depth_frames + [frame], held)
        self.publish_detections(frame, scene, "closer look: SAM 3 + depth")
        return frame

    def look_down_at(self, x, y, height, world, held=None):
        """A frame looking straight down at (x, y) from `height` above the
        table, or None when no such view is reachable."""
        # The camera is beside the gripper: turning the view about the
        # vertical moves the gripper around the item. Try first the turns that
        # put the gripper farthest from the robot's base axis: near the base
        # the arm folds over its own shoulder, low over the table.
        ups = [(math.cos(a), math.sin(a), 0.0) for a in np.arange(8) * math.pi / 4]
        T_cam_grip = inv_tf(self.views.T_grip_cam)

        def grip_r(up):
            T = down_view(x, y, height, up) @ T_cam_grip
            return -math.hypot(T[0, 3], T[1, 3])

        ups.sort(key=grip_r)
        for up in ups:
            try:
                return self.look(down_view(x, y, height, up), world, held, "look closer")
            except PlanError:
                continue
        return None

    def map_depth(self, scene, frames, held=None):
        """scene.depth: the obstacle model from these frames (depth_world.py)."""
        sk = self.views.sk
        held_box = (lambda q: sk.held_box(held, sk.P.kin.fk(q))) if held is not None else None
        scene.depth_frames = list(frames)
        prior = self.held.depth_before if held is not None and self.held is not None else None
        scene.depth = DepthWorld(frames, self.cell, self.items, self.basket, scene.basket_xy, scene.detections,
                                 max_item_height=self.cfg["max_item_height"], robot_radius=self.cfg["robot_radius"],
                                 held_box=held_box, mask_erode_px=int(self.cfg["sam3"]["mask_erode_px"]),
                                 prior=prior)
        scene.obstacles = scene.depth.blobs()
        if self.run_dir:
            os.makedirs(self.run_dir, exist_ok=True)
            cv2.imwrite(os.path.join(self.run_dir, f"{time.strftime('%H%M%S')}_depth_map.png"), scene.depth.image())

    def publish_detections(self, frame, scene, title):
        """The frame annotated with what was found in it (masks, the items'
        estimated models, names and scores, the basket, obstacles)."""
        table_z = self.cell["table"]["surface_z"]
        self.show(frame, lambda v: annotate.draw_perception(v, scene, self.items, self.basket, table_z, title), title)

    def show(self, frame, draw, title):
        """An annotated observation: draw(view) -> BGR image, drawn on the
        frame's colour image and on its depth (annotate.DepthLook)."""
        from ..viewer import depth_legend
        vis = draw(frame)
        look = annotate.DepthLook(frame)
        self.publish_image(vis, title, depth_legend(draw(look), look.lo, look.hi))

    def publish_image(self, vis, title, depth_vis=None):
        """A BGR image on /lab_skills/detections; into run_dir (beside its
        depth twin) and to on_observation (the console's camera window)."""
        from sensor_msgs.msg import Image
        msg = Image(height=vis.shape[0], width=vis.shape[1], encoding="bgr8", step=vis.shape[1] * 3,
                    data=vis.tobytes())
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.det_pub.publish(msg)
        if self.on_observation is not None:
            try:
                self.on_observation(vis, depth_vis, title)
            except Exception as e:  # noqa: BLE001
                self.log(f"  camera window: {e}")
        if self.run_dir:
            os.makedirs(self.run_dir, exist_ok=True)
            name = "".join(c if c.isalnum() or c in "-_" else "_" for c in title)
            both = vis if depth_vis is None or depth_vis.shape != vis.shape else np.hstack([vis, depth_vis])
            cv2.imwrite(os.path.join(self.run_dir, f"{time.strftime('%H%M%S')}_{name}.jpg"), both)

    def best_frame(self, scene):
        """The scene's frame of the highest-weight view (the top view)."""
        views = self.cfg["survey_views"] + self.cfg.get("recognition_views", [])
        k = int(np.argmax([float(v.get("weight", 1.0)) for v in views[:len(scene.frames)]]))
        return scene.frames[k]

    def save_scene(self, scene, title):
        """The perceived placements as YAML in run_dir (for debugging)."""
        if not self.run_dir:
            return
        import yaml
        os.makedirs(self.run_dir, exist_ok=True)
        data = {"basket_xy": [float(v) for v in scene.basket_xy],
                "items": {k: dict(d.estimate.placement.as_dict(), score=round(float(d.score), 3),
                                  notes=d.estimate.notes) for k, d in scene.detections.items()},
                "obstacles": [{"polygon": o.polygon.round(3).tolist(), "z_top": round(o.z_top, 3),
                               "support": o.support, "unseen": o.unseen} for o in getattr(scene, "obstacles", [])]}
        with open(os.path.join(self.run_dir, f"{time.strftime('%H%M%S')}_{title}.yaml"), "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    # ---- planning context -------------------------------------------------------------

    def skills_for(self, scene):
        return PerceivedSkills(self.cell, self.items, self.basket, basket_xy=scene.basket_xy,
                               depth=getattr(scene, "depth", None))

    def table_region(self, basket_xy, depth=None):
        """Where an item may be put on the table: the placement region of
        cell.yaml, and with `depth` only where the camera's views cover the
        item up to its top. Nearer the edge of the view the item would cast
        its shadow into space no view sees, and could not be picked up again."""
        pl, t = self.cell["placement"], self.cell["table"]
        m = pl["edge_margin"]
        region = dict(x_range=(max(pl["x_range"][0], t["x_min"] + m), min(pl["x_range"][1], t["x_max"] - m)),
                      y_range=(max(pl["y_range"][0], t["y_min"] + m), min(pl["y_range"][1], t["y_max"] - m)),
                      keepout=[(0.0, 0.0, pl["robot_keepout_radius"]),
                               (basket_xy[0], basket_xy[1], self.basket.r_top + pl["basket_margin"])])
        if depth is not None:
            region["accept"] = lambda fp, top: depth.in_view(fp, top + 0.01)
        return region

    def assumed_held(self, item):
        """How `item` is probably held when it was not picked by this
        executive: its first resting pose's grasp nearest to the measured width."""
        obj = self.items[item]
        w = self.width()
        best = None
        for rest in obj.spec["stable_poses"]:
            for g in obj.poses[rest].grasps:
                d = abs(g.width - w)
                if best is None or d < best[0]:
                    best = (d, rest, g)
        _, rest, g = best
        yaw_rel = wrap(math.pi / 2 - (g.yaws[0] if not g.continuous else 0.0))
        return HeldItem(item, Held(item, rest, g.width, g.tip_z, tuple(g.center), yaw_rel, obj.poses[rest].top),
                        assumed=True)

    # ---- the actions --------------------------------------------------------------------------

    def execute(self, action, target_xy=None):
        """Run one action. Returns (success, message)."""
        name, item = parse_action(action)
        if item not in self.items:
            raise ActionError(f"unknown item {item!r}")
        t0 = time.time()
        self.log(f"== {action}")
        self.action = action
        try:
            if name == "pick-up":
                msg = self.pick_up(item)
            else:
                msg = self.put(item, "basket" if name == "put-in-basket" else "table", target_xy)
        except (ActionError, PlanError) as e:
            self.phase(f"failed: {e}")
            return False, str(e)
        self.phase(f"done: {action} ({time.time() - t0:.0f} s)")
        return True, msg

    def pick_up(self, item):
        if self.holding():
            raise ActionError(f"not applicable: the gripper is holding "
                              f"{self.held.item if self.held else 'something'}")
        self.set_held(None)
        scene = self.survey(target=item)
        if item not in scene.detections:
            raise ActionError(f"{item} not found")
        close = self.close_look(scene, item)
        p = scene.detections[item].estimate.placement
        self.log(f"  {item}: {p.rest} on the {p.support} at ({p.x:.3f}, {p.y:.3f}), yaw "
                 f"{math.degrees(p.yaw):.0f} deg")
        self.phase(f"planning the grasp of {item}")
        sk = self.skills_for(scene)
        world = scene.world()
        q = self.node.current()
        pre = self.to_topdown(scene, q, world, self.width())
        first = [self.traj("level the gripper", pre)]
        try:
            steps, q_end, held = sk.plan_pick(pre[-1], item, world, max_candidates=32)
        except PlanError as e:
            # From another arm configuration the grasp may be reachable:
            # try again from home (the generator plans from there).
            self.log(f"  {e}; trying from home")
            home_steps, q_home = sk.plan_home(pre[-1], world)
            steps, q_end, held = sk.plan_pick(q_home, item, world, max_candidates=32)
            first += home_steps
        open_w = next(s["width"] for s in steps if s.get("label") == "open")
        self.show(close if close is not None else self.best_frame(scene),
                  lambda v: annotate.draw_grasp(v, self.items[item], p, held, open_w, sk.force,
                                                f"grasp {item}"), f"grasp {item}")
        self.phase(f"picking up {item}")
        try:
            self.run(first + steps)
        except Exception as e:
            self.recover(scene)
            raise ActionError(f"grasp failed: {e}")
        self.set_held(HeldItem(item, held, p, depth_before=scene.depth))
        if not self.holding():
            if abs(self.width() - held.width) < 0.012:
                self.log(f"  grip flag off but the fingers are {self.width() * 1000:.0f} mm apart: holding {item}")
            else:
                self.recover(scene)
                self.set_held(None)
                raise ActionError(f"{item} slipped out of the gripper")
        if self.return_home:
            self.go_home(sk, {k: v for k, v in world.items() if k != item}, held)
        return f"holding {item} (width {self.width() * 1000:.0f} mm)"

    def put(self, item, support, target_xy=None):
        if not self.holding():
            raise ActionError("not applicable: the gripper is empty")
        if self.held is None or self.held.item != item:
            if self.held is not None and not self.held.assumed:
                raise ActionError(f"the gripper holds {self.held.item}, not {item}")
            self.set_held(self.assumed_held(item))
            self.log(f"  assuming {item} is held at its nominal grasp")
        held = self.held.held
        scene = self.survey(exclude=(item,), held=held)
        self.phase(f"planning where to put {item}")
        sk = self.skills_for(scene)
        world = scene.world(exclude=(item,))
        q = self.node.current()
        pre = self.to_topdown(scene, q, world, held.width, held=held)
        try:
            tgt, steps = self._plan_put(sk, pre[-1], held, world, support, scene, target_xy)
            first = [self.traj("level the gripper", pre)]
        except PlanError as e:
            if "no free spot" in str(e):
                raise
            self.log(f"  {e}; trying from home")
            home_steps, q_home = sk.plan_home(pre[-1], world, held=held)
            tgt, steps = self._plan_put(sk, q_home, held, world, support, scene, target_xy)
            first = [self.traj("level the gripper", pre)] + home_steps
        self.log(f"  putting {item} at ({tgt.x:.3f}, {tgt.y:.3f}) on the {support}")
        self.phase(f"putting {item} {'in the basket' if support == 'basket' else 'on the table'}")
        self.run(first + steps)
        self.spin(0.3)
        if self.holding():
            raise ActionError(f"{item} is still in the gripper after release")
        self.set_held(None)
        world[item] = tgt
        if self.return_home:
            self.go_home(sk, world, None)
        return f"put {item} at ({tgt.x:.3f}, {tgt.y:.3f}) on the {support}"

    def _plan_put(self, sk, q0, held, world, support, scene, target_xy):
        """(target placement, steps) for putting the held item down."""
        if support == "basket":
            return self._plan_drop(sk, q0, held, world, scene)
        region = self.table_region(scene.basket_xy, getattr(scene, "depth", None))
        prefer = target_xy if target_xy is not None and len(target_xy) == 2 else (
            (self.held.source.x, self.held.source.y) if self.held.source is not None else (0.45, -0.10))
        tgt, steps, q_end = sk.find_spot(q0, held, "table", world, region=region, prefer=tuple(prefer),
                                         max_tries=self.cfg.get("put_max_tries", 80))
        fp = self.items[held.item].footprint_world(held.rest, tgt.x, tgt.y, tgt.yaw)
        self.show(self.best_frame(scene),
                  lambda v: annotate.draw_table(v, self.cell, self.items, self.basket, scene, region, fp, tgt,
                                                self.held.source, held.item, f"put {held.item} here (free spot)"),
                  "put on the table")
        return tgt, steps

    def _plan_drop(self, sk, q0, held, world, scene):
        """Drop the held item above the emptiest spot of the basket (it may
        land on what is there). Only the motion can fail, so the next best
        spots are tried."""
        fp = self.items[held.item].poses[held.rest].footprint
        yaws = [wrap(k * math.pi / 4 + held.yaw_rel) for k in range(-4, 4)]
        drops = drop_spots(scene.frames, self.cell, self.basket, scene.basket_xy, fp, yaws,
                           depth=scene.depth, items=list(world))
        err = None
        # The best spots with the yaw that fits there, then any yaw.
        tries = [(s, [s[3]]) for s in spread(drops.spots)] + [(s, None) for s in spread(drops.spots, 4)]
        for (cost, x, y, yaw, z_pile), only in tries:
            try:
                tgt, steps, _ = sk.plan_drop(q0, held, x, y, z_pile, world, yaws=only)
            except PlanError as e:
                err = e
                continue
            rel = next(s for s in steps if s.get("label") == "release")
            fpw = self.items[held.item].footprint_world(held.rest, tgt.x, tgt.y, tgt.yaw)
            z_drop = tgt.z_support + rel["drop"]
            pile = z_pile - drops.z_floor
            self.log(f"  dropping from {z_drop * 100:.0f} cm onto "
                     + ("the floor" if pile < 0.01 else f"a pile {pile * 100:.0f} cm high"))
            self.show(self.best_frame(scene),
                      lambda v: annotate.draw_drop(v, self.basket, scene.basket_xy, self.cell["table"]["surface_z"],
                                                   drops, fpw, tgt, z_drop, held.item,
                                                   f"drop {held.item}: emptiest spot"),
                      "drop in the basket")
            return tgt, steps
        raise err or PlanError("no drop spot in the basket")

    def to_topdown(self, scene, q, world, width, held=None):
        """From a camera view back to a top-down gripper pose, clear of what was seen."""
        self.views.sk.depth = scene.depth
        try:
            return self.views.to_topdown(q, world, width, held=held)
        finally:
            self.views.sk.depth = None

    # ---- items added at run time ------------------------------------------------------------

    def add_item(self, name, spec):
        """Know `name` from now on: its model (objects/measured.py) and its SAM 3 prompts."""
        self.items[name] = objects.make_item(name, spec)
        self.perception.add_prompts(name, spec["prompts"])

    def forget_item(self, name):
        self.items.pop(name, None)
        self.perception.remove_prompts(name)

    def new_item(self, name, prompt, shape=None):
        """Find an item that is not known yet on the table by its text
        prompt, measure it and add it (add_item). Returns (success, message,
        spec); the spec is an objects/measured.py spec."""
        t0 = time.time()
        self.log(f"== new item {name}: {prompt!r}")
        self.action = f"new-item {name}"
        try:
            spec, msg = self._measure_new(name, prompt, shape)
        except (ActionError, PlanError) as e:
            self.phase(f"failed: {e}")
            return False, str(e), None
        self.add_item(name, spec)
        self.phase(f"done: new item {name} ({time.time() - t0:.0f} s)")
        return True, msg, spec

    def _find_new(self, frame, prompt, known, basket_xy, near=None):
        """(SAM score, mask, world points) of the best mask of `prompt` in
        `frame` on the table, or None. Masks of known items that SAM 3 named
        better are skipped; `near` (x, y) keeps only masks within 5 cm of it."""
        from ..perception.scene import _overlap
        res = self.perception.sam.segment(frame.rgb, [prompt], threshold=self.cfg["sam3"]["threshold"])[prompt]
        best = None
        for inst in res:
            if any(d.frame is frame and d.sam_score >= inst.score and _overlap(d.mask, inst.mask) > 0.5
                   for d in known.values()):
                continue
            # Not eroded: the size is read from the mask's outline (eroding
            # 1 px made a 60 x 30 mm block 56 x 27 mm in simulation).
            pts = self.perception._points(frame, inst.mask, erode_px=0)
            pts = pts[pts[:, 2] > self.cell["table"]["surface_z"] + 0.004]
            if len(pts) < 30:
                continue
            c = np.median(pts[:, :2], axis=0)
            if basket_xy is not None and math.hypot(*(c - basket_xy)) < self.basket.r_top + 0.01:
                self.log(f"  {prompt!r} (score {inst.score:.2f}) is in the basket: skipped")
                continue
            if near is not None and math.hypot(*(c - np.asarray(near))) > 0.05:
                continue
            if best is None or inst.score > best[0]:
                best = (inst.score, inst.mask, pts)
        return best

    def _measure_new(self, name, prompt, shape):
        from ..perception.estimate import estimate_item, measure_item
        if self.holding():
            raise ActionError("the gripper is holding something: put it down first, then measure the new item")
        scene = self.survey()
        self.phase(f"looking for {prompt!r} (SAM 3)")
        found = None
        for f in scene.frames:
            hit = self._find_new(f, prompt, scene.detections, scene.basket_xy)
            if hit is not None and (found is None or hit[0] > found[0]):
                found = hit + (f,)
        if found is None:
            raise ActionError(f"nothing on the table looks like {prompt!r} to SAM 3 "
                              f"(try another description: prompt=\"...\")")
        score, mask, pts, frame = found
        c = np.median(pts[:, :2], axis=0)
        self.log(f"  {prompt!r} found with score {score:.2f} at ({c[0]:.3f}, {c[1]:.3f})")
        notes = []
        cl = self.cfg.get("close_look", {})
        close = self.look_down_at(c[0], c[1], cl.get("height", 0.5), scene.world())
        if close is not None:
            self.phase(f"measuring {name} from closer")
            hit = self._find_new(close, prompt, {}, scene.basket_xy, near=c)
            if hit is not None:
                score, mask, pts = hit
                frame = close
            else:
                notes.append("not found again from closer: measured from the survey")
        else:
            notes.append("no closer view reachable: measured from the survey")
        table_z = self.cell["table"]["surface_z"]
        m = measure_item(pts, table_z, shape)
        if m is None:
            raise ActionError(f"too few depth points on {prompt!r} to measure it")
        shape, size, more = m
        notes += more
        width, height = (size[0] if shape == "round" else size[1]), size[-1]
        if width > rg6.MAX_WIDTH - 0.01:
            raise ActionError(f"{name} is {width * 1000:.0f} mm wide, too wide for the RG6 "
                              f"(it opens {rg6.MAX_WIDTH * 1000:.0f} mm)")
        if height > self.cfg["max_item_height"]:
            raise ActionError(f"{name} is {height * 100:.0f} cm tall; items may be at most "
                              f"{self.cfg['max_item_height'] * 100:.0f} cm (perception.yaml max_item_height)")
        if height < 0.015:
            notes.append(f"only {height * 1000:.0f} mm tall: the pads stop 8 mm above the table "
                         f"(objects/base.py TIP_CLEARANCE) and hold little of it")
        color = np.median(frame.rgb[mask], axis=0) / 255.0
        spec = {"kind": "measured", "shape": shape, "size": [float(v) for v in size], "mass": 0.1,
                "color": [round(float(v), 3) for v in color],
                "stable_poses": ["upright" if shape == "round" else "flat"], "prompts": [prompt],
                "measured": time.strftime("%Y-%m-%d %H:%M")}
        # The new model must explain the points it was made from.
        model = objects.make_item(name, spec)
        est = estimate_item(model, pts, table_z, self.basket, scene.basket_xy)
        if est is None or est.plausibility < 0.5:
            notes.append(f"its model fits its own points badly (plausibility "
                         f"{0 if est is None else est.plausibility:.2f})")
        dims = " x ".join(f"{v * 1000:.0f}" for v in size) + " mm"
        what = f"{shape} {dims}" + (" (diameter x height)" if shape == "round" else " (length x width x height)")
        self.show(frame, lambda v: annotate.draw_measure(v, mask, model, est.placement if est else None, dims,
                                                         f"new item {name}: {what}",
                                                         f"{prompt!r}, SAM 3 score {score:.2f}; magenta: its mask, "
                                                         f"green: the model made from it"), f"new item {name}")
        self.log(f"  {name}: {what}; " + "; ".join(notes))
        if self.return_home:
            self.go_home(self.skills_for(scene), scene.world(), None)
        msg = f"{name} is a {what}, SAM 3 score {score:.2f}"
        return spec, msg + "".join(f"\n    note: {n}" for n in notes)

    def go_home(self, sk, world, held):
        self.phase("returning home")
        try:
            steps, _ = sk.plan_home(self.node.current(), world, held=held)
            self.run(steps)
        except PlanError as e:
            self.log(f"  no path home ({e}); staying here")

    def recover(self, scene):
        """After a failed grasp: open, rise straight up."""
        self.phase("recovering: open and rise")
        try:
            self.node.gripper(min(self.width() + 0.03, rg6.MAX_WIDTH), self.views.sk.force, "open")
            q = self.node.current()
            path = self.views._rise(q, self.views.P.kin.fk(q)[2, 3] + 0.12)
            self.run([self.views.sk._traj("rise", path, slow=True)])
        except Exception as e:  # noqa: BLE001
            self.log(f"  recovery failed: {e}")
