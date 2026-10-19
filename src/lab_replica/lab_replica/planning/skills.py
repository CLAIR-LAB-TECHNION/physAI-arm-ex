"""Pick and place skills for the PDDL actions, as collision-checked motion plans.

A plan is a list of steps:
  {"kind": "traj", "label": str, "points": [6-list, ...], "times": [s, ...]}
  {"kind": "gripper", "label": str, "width": m, "force": N, "expect_grip": bool}

`world` arguments are dicts {item: Placement} describing where the objects
that are not in the gripper currently rest. The same code checks feasibility
for the problem generator and produces the hard-coded solutions, so anything
the generator accepts can be executed.
"""

import math
from dataclasses import dataclass

import numpy as np

from ..robot import rg6
from ..robot.collision import RobotPoints, object_box
from ..robot.kinematics import robot_urdf
from ..transforms import rpy_to_matrix, wrap
from .geometry2d import polygon_inside_circle, polygons_intersect
from .motion import PlanError, Planner, gripper_pose, psi_for_closing
from .placement import Placement, placement_pose

OPEN_MARGIN = 0.016         # extra opening on each side of the object before closing
SQUEEZE = 0.025             # close this much below the object width (force limited)
RELEASE_GAP_TABLE = 0.004   # object bottom above the table when released
RELEASE_GAP_BASKET = 0.012  # object bottom above its support in the basket
DROP_CLEARANCE = 0.02       # dropping into the basket: object bottom above the rim or the pile below
TRANSIT_Z = 0.40            # lowest transit height of the RG6 mounting face
TRANSIT_Z_MAX = 0.74        # the 27 cm spray bottle, held at its head, needs ~0.69 to be carried over the basket rim
MARGIN = 0.008              # clearance to obstacles for collision checks
WALL_MARGIN = 0.03          # clearance to the wall behind the table (cell.yaml room.wall_y)


@dataclass
class Held:
    item: str
    rest: str
    width: float          # object width between the pads
    tip_z: float          # fingertip height above the object's support at grasp
    offset_obj: tuple     # grasp centre in the rested frame (x, y)
    yaw_rel: float        # object yaw minus gripper yaw
    top: float            # object height in this resting pose


class Skills:
    """Motion plans for the PDDL actions in one cell (cell.yaml) with the given
    item models and basket."""

    def __init__(self, cell, items, basket, basket_xy=None):
        self.cell = cell
        self.items = items
        self.basket = basket
        self.basket_xy = np.array(basket_xy if basket_xy is not None else cell["basket"]["center_xy"], float)
        self.P = Planner(cell)
        self.robot = RobotPoints(robot_urdf(cell["robot"]["ur_type"]))
        self.force = float(cell["gripper"]["grip_force"])
        self.table_z = cell["table"]["surface_z"]
        self.wall_y = float(cell["room"]["wall_y"])

    # ---- world helpers --------------------------------------------------------

    def basket_floor(self):
        return self.table_z + self.basket.floor

    def footprint(self, p):
        return self.items[p.item].footprint_world(p.rest, p.x, p.y, p.yaw)

    def top_of(self, p):
        return p.z_support + self.items[p.item].poses[p.rest].top

    def boxes(self, world, exclude=()):
        out = []
        for item, p in world.items():
            if item in exclude:
                continue
            xyz, R = placement_pose(self.items[item], p)
            out.append(object_box(self.items[item], xyz, R, item))
        return out

    def stack_on(self, world, support):
        """[(footprint, top_z)] of the objects resting on a support."""
        return [(self.footprint(p), self.top_of(p)) for p in world.values() if p.support == support]

    def top_under(self, poly, world, support=None, margin=0.0):
        """Highest top (world z) of the objects in `world` (on `support`, or
        on any) that come within `margin` of the 2D polygon; -inf if none."""
        tops = [self.top_of(p) for p in world.values() if (support is None or p.support == support)
                and polygons_intersect(poly, self.footprint(p), margin)]
        return max(tops) if tops else -math.inf

    def held_box(self, held, T_grip):
        """Box of the held object for a gripper pose. The object moves rigidly
        with the gripper: it was grasped top-down, and when the gripper leans
        (e.g. for a camera view) the object leans with it."""
        obj = self.items[held.item]
        rest = obj.poses[held.rest]
        # Object pose in the gripper frame for a top-down grasp at yaw 0.
        depth = rg6.depth_from_width(held.width)
        c, s = math.cos(held.yaw_rel), math.sin(held.yaw_rel)
        ox, oy = held.offset_obj
        p0 = np.array([-(c * ox - s * oy), -(s * ox + c * oy), -depth - held.tip_z + rest.z])
        R0 = rpy_to_matrix(0, 0, held.yaw_rel) @ rpy_to_matrix(*rest.rpy)
        G0 = rpy_to_matrix(math.pi, 0, 0)       # top-down gripper at yaw 0 (motion.R_DOWN)
        R = T_grip[:3, :3] @ G0.T @ R0
        p = T_grip[:3, 3] + T_grip[:3, :3] @ (G0.T @ p0)
        return object_box(obj, p, R, held.item)

    # ---- collision checking ------------------------------------------------------

    def check_path(self, path, width, world, exclude=(), held=None, fingers_may_touch=(), label="",
                   held_in_basket=False):
        """Raise PlanError if the robot (or the held object) collides with the
        table, the wall behind it, the basket or an object anywhere along the
        path. The held object may go below the basket rim inside the basket
        only with held_in_basket (placing it there, or picking it from there)."""
        boxes = self.boxes(world, exclude=set(exclude) | ({held.item} if held else set()))
        bx, by = self.basket_xy
        b = self.basket
        rim = self.table_z + b.height
        # Points on the held object's box: corners, edge midpoints and faces.
        g = np.stack(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, -0.5, 0, 0.5, 1]), -1).reshape(-1, 3).astype(float)
        for q in path:
            arm, fing = self.robot.world_points(q, width)
            hold = None
            if held is not None:
                hb = self.held_box(held, self.P.kin.fk(q))
                hold = hb.c + (g * hb.h) @ hb.R.T
            # Table.
            if arm[:, 2].min() < self.table_z + 0.006:
                raise PlanError(f"{label}: arm touches the table")
            if fing[:, 2].min() < self.table_z + 0.001:
                raise PlanError(f"{label}: fingers touch the table")
            if hold is not None and hold[:, 2].min() < self.table_z - 0.002:
                raise PlanError(f"{label}: held object hits the table")
            # Wall: everything stays in front of the plane y = wall_y.
            y_lim = self.wall_y - WALL_MARGIN
            if arm[:, 1].max() > y_lim or fing[:, 1].max() > y_lim:
                raise PlanError(f"{label}: robot reaches the wall")
            if hold is not None and hold[:, 1].max() > y_lim:
                raise PlanError(f"{label}: held object reaches the wall")
            if hold is not None and not held_in_basket:
                r = np.hypot(hold[:, 0] - bx, hold[:, 1] - by)
                if np.any((r < b.r_top + MARGIN) & (hold[:, 2] < rim + MARGIN)):
                    raise PlanError(f"{label}: held object goes into the basket")
            # Basket: the arm stays above the rim; fingers and the held object
            # may go inside but must keep off the wall.
            r_arm = np.hypot(arm[:, 0] - bx, arm[:, 1] - by)
            r_out = b.r_bottom + b.slope * np.clip(arm[:, 2] - self.table_z, 0, b.height) + 0.015
            if np.any((r_arm < r_out) & (arm[:, 2] < rim + 0.015)):
                raise PlanError(f"{label}: arm enters the basket")
            for P in [fing] + ([hold] if hold is not None else []):
                r = np.hypot(P[:, 0] - bx, P[:, 1] - by)
                z = P[:, 2] - self.table_z
                r_in = b.r_bottom - b.wall + b.slope * np.clip(z, 0, None)
                if np.any((r > r_in - MARGIN) & (r < r_in + b.wall + MARGIN) & (z < b.height + MARGIN)):
                    raise PlanError(f"{label}: gripper or object hits the basket wall")
                if np.any((r < r_in) & (z < b.floor - 0.001)):
                    raise PlanError(f"{label}: below the basket floor")
            # Objects.
            for box in boxes:
                if np.any(box.inside(arm, MARGIN)):
                    raise PlanError(f"{label}: arm hits {box.name}")
                if box.name not in fingers_may_touch and np.any(box.inside(fing, MARGIN * 0.5)):
                    raise PlanError(f"{label}: fingers hit {box.name}")
                if hold is not None and np.any(box.inside(hold, 0.004)):
                    raise PlanError(f"{label}: held object hits {box.name}")

    # ---- transit ------------------------------------------------------------------

    def _transit(self, q0, goal_xy, psi_goal, z_min, width, world, held=None, label="transit",
                 held_in_basket=False):
        """Rise vertically, then move around the base at a safe height to
        above goal_xy. Tries increasing heights until collision free."""
        P = self.P
        T0 = P.kin.fk(q0)
        p0 = T0[:3, 3]
        psi0 = math.atan2(T0[1, 0], T0[0, 0])
        err = None
        for z in np.arange(max(z_min, TRANSIT_Z), TRANSIT_Z_MAX + 1e-6, 0.04):
            # Either change height while travelling (one arc, which also works
            # near the edge of the workspace), or rise first and then travel.
            for rise_first in (False, True):
                try:
                    path = [np.asarray(q0, float)]
                    z0 = p0[2]
                    if rise_first and p0[2] < z - 1e-3:
                        path = P.line(q0, p0, (p0[0], p0[1], z), psi0, psi0)
                        z0 = z
                    arc = P.arc(path[-1], (p0[0], p0[1], z0), (goal_xy[0], goal_xy[1], z), psi0, psi_goal)
                    path += arc[1:]
                    self.check_path(path, width, world, held=held, label=label, held_in_basket=held_in_basket)
                    return path, z
                except PlanError as e:
                    err = e
        # Last resort: a straight joint-space move to the configuration above
        # the goal, if it is collision free.
        for z in np.arange(max(z_min, TRANSIT_Z), TRANSIT_Z_MAX + 1e-6, 0.04):
            q_goal = P.ik(gripper_pose(goal_xy[0], goal_xy[1], z, psi_goal), q0)
            if q_goal is None:
                continue
            path = P.joint_move(q0, q_goal, step=0.02)
            try:
                self.check_path(path, width, world, held=held, label=label, held_in_basket=held_in_basket)
                return path, z
            except PlanError as e:
                err = e
        raise err or PlanError(f"{label}: no transit height works")

    # ---- pick -----------------------------------------------------------------

    def _grasp_options(self, item, p):
        """Candidate (x, y, psi, width, tip_z, center) for a placement."""
        obj = self.items[item]
        rest = obj.poses[p.rest]
        out = []
        for g in rest.grasps:
            cx, cy = g.center
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            gx, gy = p.x + c * cx - s * cy, p.y + s * cx + c * cy
            if g.continuous:
                bearing = math.atan2(gy - self.P.base_xy[1], gx - self.P.base_xy[0])
                yaws = [bearing + k * math.pi / 6 for k in range(6)]
            else:
                yaws = [p.yaw + y for y in g.yaws]
            for cy_ in yaws:
                for psi in psi_for_closing(cy_):
                    out.append((gx, gy, psi, g.width, g.tip_z, g.center))
        return out

    def finger_boxes(self, x, y, psi, width, open_w):
        """Footprints (2D rectangles) of the two fingers opened to open_w."""
        d = np.array([math.cos(psi + math.pi / 2), math.sin(psi + math.pi / 2)])  # closing dir
        t = np.array([-d[1], d[0]])
        boxes = []
        for side in (-1, 1):
            c = np.array([x, y]) + side * d * (open_w / 2 + rg6.FINGER_THICKNESS / 2)
            hw, hd = rg6.FINGER_HALF_WIDTH + 0.003, rg6.FINGER_THICKNESS / 2 + 0.003
            corners = [c + a * hd * d + b * hw * t for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            boxes.append(np.array(corners))
        return boxes

    def plan_pick(self, q0, item, world, max_candidates=8):
        """Plan approach, grasp and lift of `item` (resting as world[item]),
        trying the `max_candidates` cheapest grasps. Returns (steps, q_end, held)."""
        P = self.P
        p = world[item]
        others = {k: v for k, v in world.items() if k != item}
        cands = []
        for gx, gy, psi, w, tip_z, center in self._grasp_options(item, p):
            if w > rg6.MAX_WIDTH - 0.01:
                continue
            open_w = min(w + 2 * OPEN_MARGIN, rg6.MAX_WIDTH)
            z_grasp = p.z_support + tip_z + rg6.depth_from_width(w)
            tips_open = z_grasp - rg6.depth_from_width(open_w)
            # The knuckles reach into the gap between the pads; the object's
            # top must stay below them, open (descending) and closed.
            top = self.top_of(p)
            if (top - (z_grasp - rg6.depth_from_width(w)) > rg6.free_height(w) - 0.002 or
                    top - tips_open > rg6.free_height(open_w) - 0.002):
                continue
            fb = self.finger_boxes(gx, gy, psi, w, open_w)
            if any(self.top_under(b, others, p.support) > tips_open - 0.01 for b in fb):
                continue
            q_g = P.ik(gripper_pose(gx, gy, z_grasp, psi), q0)
            if q_g is None:
                continue
            cost = np.abs(q_g - q0).max() + 0.25 * abs(q_g[5])
            cands.append((cost, gx, gy, psi, w, tip_z, center, open_w, z_grasp))
        cands.sort(key=lambda c: c[0])
        errors = [] if cands else ["no reachable grasp"]
        for _, gx, gy, psi, w, tip_z, center, open_w, z_grasp in cands[:max_candidates]:
            held = Held(item, p.rest, w, tip_z, tuple(center), wrap(p.yaw - psi),
                        self.items[item].poses[p.rest].top)
            try:
                steps, q_end = self._pick_motion(q0, item, world, gx, gy, psi, z_grasp, w, open_w, held)
                return steps, q_end, held
            except PlanError as e:
                errors.append(str(e))
        raise PlanError(f"cannot pick {item}: " + "; ".join(sorted(set(errors))[:3]))

    def _pick_motion(self, q0, item, world, gx, gy, psi, z_grasp, w, open_w, held):
        P = self.P
        steps = []
        path, z_top = self._transit(q0, (gx, gy), psi, z_grasp + 0.12, open_w, world, label="move above grasp")
        steps.append(self._traj("move above grasp", path))
        steps.append(dict(kind="gripper", label="open", width=open_w, force=self.force, expect_grip=False))
        down = P.line(path[-1], (gx, gy, z_top), (gx, gy, z_grasp), psi, psi, step=0.01)
        self.check_path(down, open_w, world, fingers_may_touch=(item,), label="descend")
        steps.append(self._traj("descend", down, slow_last=0.06))
        steps.append(dict(kind="gripper", label="grasp", width=max(0.0, w - SQUEEZE), force=self.force,
                          expect_grip=True))
        # Lift high enough that the held object clears its neighbours too.
        others = {k: v for k, v in world.items() if k != item}
        err = None
        for z_lift in np.arange(z_top, TRANSIT_Z_MAX + 1e-6, 0.04):
            up = P.line(down[-1], (gx, gy, z_grasp), (gx, gy, z_lift), psi, psi, step=0.01)
            try:
                self.check_path(up[2:], w, others, label="lift")
                self.check_path(up[3:], w, others, held=held, label="lift",
                                held_in_basket=world[item].support == "basket")
                steps.append(self._traj("lift", up, slow_first=0.06))
                return steps, up[-1]
            except PlanError as e:
                err = e
        raise err

    # ---- place ------------------------------------------------------------------

    def object_xy_for(self, x_obj, y_obj, yaw_obj, held: Held):
        """Gripper (x, y, psi) that puts the held object at (x, y, yaw)."""
        c, s = math.cos(yaw_obj), math.sin(yaw_obj)
        cx, cy = held.offset_obj
        return x_obj + c * cx - s * cy, y_obj + s * cx + c * cy, wrap(yaw_obj - held.yaw_rel)

    def plan_place(self, q0, held: Held, target: Placement, world, max_drop=None):
        """Carry the held object above `target`, lower it, release, retreat.

        The object is lowered until it almost touches its support. If the
        fingers would hit the basket wall or a neighbour on the way down it
        is released higher instead, up to `max_drop` above the support
        (default 9 cm in the basket, nothing on the table).
        """
        P = self.P
        gx, gy, psi = self.object_xy_for(target.x, target.y, target.yaw, held)
        in_basket = target.support == "basket"
        gap = RELEASE_GAP_BASKET if in_basket else RELEASE_GAP_TABLE
        if max_drop is None:
            max_drop = 0.09 if in_basket else 0.0
        depth = rg6.depth_from_width(held.width)
        release_w = min(held.width + 2 * OPEN_MARGIN, rg6.MAX_WIDTH)
        last = None
        for k in range(int(round(max_drop / 0.01)) + 1):
            z_tips = target.z_support + gap + held.tip_z + 0.01 * k
            z_rel = z_tips + depth
            try:
                self._check_fingers_vs(world, target, gx, gy, psi, held.width, release_w, z_tips)
                path, z_top = self._transit(q0, (gx, gy), psi, z_rel + 0.10, held.width, world,
                                            held=held, label="carry", held_in_basket=in_basket)
                down = P.line(path[-1], (gx, gy, z_top), (gx, gy, z_rel), psi, psi, step=0.01)
                self.check_path(down, held.width, world, held=held, label="lower", held_in_basket=in_basket)
                self.check_path(down[-1:], release_w, world, label="release")
                up = P.line(down[-1], (gx, gy, z_rel), (gx, gy, z_top), psi, psi, step=0.01)
                self.check_path(up, release_w, world, label="retreat")
                steps = [self._traj("carry", path),
                         self._traj("lower", down, slow_last=0.06),
                         dict(kind="gripper", label="release", width=release_w, force=self.force,
                              expect_grip=False, drop=round(0.01 * k + gap, 3)),
                         self._traj("retreat", up, slow_first=0.05)]
                return steps, up[-1]
            except PlanError as e:
                last = e
        raise last

    def plan_drop(self, q0, held: Held, x, y, z_pile, world, max_raise=0.10, yaws=None):
        """Carry the held object above (x, y) in the basket and let go of it,
        its bottom DROP_CLEARANCE above the rim or above `z_pile` (the top of
        what is below it), whichever is higher. It falls onto whatever is
        there; only the robot and the object on its way down are kept clear
        of things. The object yaws `yaws` are tried (default: those of the
        eight gripper yaws, nearest the current one first), then releases up
        to `max_raise` higher. Returns (target, steps, q_end):
        target is where the object should come to rest (on the pile)."""
        P = self.P
        rim = self.table_z + self.basket.height
        z_support = max(self.basket_floor(), z_pile)
        depth = rg6.depth_from_width(held.width)
        release_w = min(held.width + 2 * OPEN_MARGIN, rg6.MAX_WIDTH)
        T0 = P.kin.fk(q0)
        psi0 = math.atan2(T0[1, 0], T0[0, 0])
        psis = sorted((k * math.pi / 4 for k in range(-4, 4)), key=lambda p: abs(wrap(p - psi0)))
        if yaws is not None:
            psis = [wrap(y - held.yaw_rel) for y in yaws]
        last = None
        for k in range(int(round(max_raise / 0.02)) + 1):
            z_bottom = max(rim, z_pile) + DROP_CLEARANCE + 0.02 * k
            z_rel = z_bottom + held.tip_z + depth
            for psi in psis:
                yaw = wrap(psi + held.yaw_rel)
                gx, gy, _ = self.object_xy_for(x, y, yaw, held)
                try:
                    path, z_top = self._transit(q0, (gx, gy), psi, z_rel + 0.06, held.width, world,
                                                held=held, label="carry")
                    down = P.line(path[-1], (gx, gy, z_top), (gx, gy, z_rel), psi, psi, step=0.01)
                    self.check_path(down, held.width, world, held=held, label="lower")
                    self.check_path(down[-1:], release_w, world, label="release")
                    up = P.line(down[-1], (gx, gy, z_rel), (gx, gy, z_top), psi, psi, step=0.01)
                    self.check_path(up, release_w, world, label="retreat")
                except PlanError as e:
                    last = e
                    continue
                tgt = Placement(held.item, held.rest, float(x), float(y), float(yaw), support="basket",
                                z_support=float(z_support))
                steps = [self._traj("carry", path),
                         self._traj("lower", down, slow_last=0.04),
                         dict(kind="gripper", label="release", width=release_w, force=self.force,
                              expect_grip=False, drop=round(z_bottom - z_support, 3)),
                         self._traj("retreat", up, slow_first=0.05)]
                return tgt, steps, up[-1]
        raise last

    def _check_fingers_vs(self, world, target, gx, gy, psi, width, open_w, z_tips):
        for w in (width, open_w):
            for fb in self.finger_boxes(gx, gy, psi, width, w):
                if self.top_under(fb, world, target.support, 0.004) > max(z_tips - 0.012, target.z_support + 1e-6):
                    raise PlanError("fingers would hit a neighbouring object")

    def find_spot(self, q0, held: Held, support, world, region=None, prefer=None, margin=None,
                  rng=None, max_tries=300):
        """Search a placement for the held object on 'basket' or 'table' and
        plan it. region (table): dict with x_range, y_range, keepout
        circles [(x, y, r)] and optionally accept(footprint, top z) -> bool. Spots are tried by distance from `prefer`, or in
        random order when rng is given. Returns (target, steps, q_end)."""
        obj = self.items[held.item]
        rest = obj.poses[held.rest]
        if margin is None:
            # On the table keep room for the fingers around every object, so
            # all of them stay graspable.
            margin = 0.012 if support == "basket" else self.cell["placement"]["object_clearance"]
        if support == "basket":
            bx, by = self.basket_xy
            r_floor = self.basket.inner_radius(0.0) - 0.006
            xs = np.arange(bx - r_floor, bx + r_floor + 1e-9, 0.01)
            ys = np.arange(by - r_floor, by + r_floor + 1e-9, 0.01)
            z_sup = self.basket_floor()
        else:
            xs = np.arange(region["x_range"][0], region["x_range"][1] + 1e-9, 0.01)
            ys = np.arange(region["y_range"][0], region["y_range"][1] + 1e-9, 0.01)
            z_sup = self.table_z
        cands = [(x, y) for x in xs for y in ys]
        if rng is not None:
            rng.shuffle(cands)
        else:
            px, py = prefer if prefer is not None else cands[len(cands) // 2]
            cands.sort(key=lambda c: (c[0] - px) ** 2 + (c[1] - py) ** 2)
        psis = [k * math.pi / 4 for k in range(-4, 4)]
        tries = 0
        last_err = "no free spot"
        # Pass 0 looks for free floor; in the basket, pass 1 lets the object
        # rest on what is already there (it is dropped onto the pile).
        passes = (0, 1) if support == "basket" else (0,)
        for stack in passes:
            for x, y in cands:
                for psi in psis:
                    yaw = wrap(psi + held.yaw_rel)
                    fp = obj.footprint_world(held.rest, x, y, yaw)
                    if support == "basket":
                        r_top = self.basket.inner_radius(min(rest.top, self.basket.height)) - 0.004
                        if not polygon_inside_circle(fp, bx, by, min(r_floor, r_top) - 0.002):
                            continue
                    else:
                        if not all(region["x_range"][0] <= v[0] <= region["x_range"][1] and
                                   region["y_range"][0] <= v[1] <= region["y_range"][1] for v in fp):
                            continue
                        if any(math.hypot(v[0] - cx, v[1] - cy) < cr for v in fp
                               for cx, cy, cr in region.get("keepout", [])):
                            continue
                        accept = region.get("accept")
                        if accept is not None and not accept(fp, z_sup + rest.top):
                            continue
                    z_under = self.top_under(fp, world, support, margin)
                    under = z_under > -math.inf
                    if under and not stack:
                        continue
                    z_here = max(z_sup, z_under)
                    # Piles may not tower over the rim (a tall object standing on
                    # the floor may).
                    if under and z_here + rest.top > self.table_z + self.basket.height + 0.06:
                        continue
                    gx, gy, gpsi = self.object_xy_for(x, y, yaw, held)
                    if support == "table":
                        release_w = min(held.width + 2 * OPEN_MARGIN, rg6.MAX_WIDTH)
                        fingers = self.finger_boxes(gx, gy, gpsi, held.width, release_w)
                        if any(self.top_under(f, world, support, margin) > -math.inf for f in fingers):
                            continue
                    tries += 1
                    if tries > max_tries:
                        raise PlanError(f"no placement for {held.item} on the {support}: {last_err}")
                    tgt = Placement(held.item, held.rest, float(x), float(y), float(yaw), support=support,
                                    z_support=float(z_here))
                    try:
                        steps, q_end = self.plan_place(q0, held, tgt, world)
                        return tgt, steps, q_end
                    except PlanError as e:
                        last_err = str(e)
        raise PlanError(f"no placement for {held.item} on the {support}: {last_err}")

    # ---- home -------------------------------------------------------------------

    def plan_home(self, q0, world, held=None):
        P = self.P
        Th = P.kin.fk(P.home)
        ph = Th[:3, 3]
        psih = math.atan2(Th[1, 0], Th[0, 0])
        width = held.width if held else rg6.width_from_q(-0.3)
        path, _ = self._transit(q0, ph[:2], psih, ph[2], width, world, held=held, label="home")
        tail = P.joint_move(path[-1], P.home)[1:]
        self.check_path(tail, width, world, held=held, label="home")
        return [self._traj("home", path + tail)], np.array(P.home)

    def _traj(self, label, path, slow=False, slow_last=None, slow_first=None):
        """Timed trajectory. slow_last / slow_first: length (m) at the end or
        start of a straight move that runs at approach speed."""
        P = self.P
        if slow_last or slow_first:
            pts = [P.kin.fk(q)[:3, 3] for q in path]
            n = len(path)
            if slow_last:
                k = next((i for i in range(n) if np.linalg.norm(pts[i] - pts[-1]) <= slow_last + 1e-9), n - 1)
                fast, slow_part = path[:k + 1], path[k:]
            else:
                k = next((i for i in range(n) if np.linalg.norm(pts[i] - pts[0]) >= slow_first - 1e-9), n - 1)
                slow_part, fast = path[:k + 1], path[k:]
            t_fast = P.time_path(fast) if len(fast) > 1 else [0.0]
            t_slow = P.time_path(slow_part, speed=P.V_SLOW) if len(slow_part) > 1 else [0.0]
            if slow_last:
                times = t_fast + [t_fast[-1] + t for t in t_slow[1:]]
            else:
                times = t_slow + [t_slow[-1] + t for t in t_fast[1:]]
        else:
            times = P.time_path(path, speed=P.V_SLOW if slow else None)
        return dict(kind="traj", label=label, points=[list(map(float, q)) for q in path], times=times)


def compress_path(planner, path, tol=0.004, return_indices=False):
    """Keep only the waypoints needed so that joint-space interpolation
    between consecutive kept points stays within `tol` of the original
    Cartesian path (checked at the dropped points)."""
    if len(path) <= 2:
        return list(range(len(path))) if return_indices else list(path)
    P = [planner.kin.fk(q)[:3, 3] for q in path]
    kept = [0]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            ok = True
            for k in range(i + 1, j):
                s = (k - i) / (j - i)
                q = (1 - s) * np.asarray(path[i]) + s * np.asarray(path[j])
                if np.linalg.norm(planner.kin.fk(q)[:3, 3] - P[k]) > tol:
                    ok = False
                    break
            if ok:
                break
            j -= 1
        kept.append(j)
        i = j
    return kept if return_indices else [path[k] for k in kept]
