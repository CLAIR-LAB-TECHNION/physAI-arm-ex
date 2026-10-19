"""Random problems, and the motion plan of a problem.

A problem is a stable arrangement of the twelve items (some on the table,
some in the basket, possibly one in the gripper) plus a PDDL goal whose
optimal plan has between min_actions and max_actions actions. Every action of
that plan is motion-planned with the collision-checked skills, so accepted
goals are physically reachable, not only reachable in PDDL.

A problem is passed around as a dict:
    placements  [Placement]  where every item rests (at its settled pose)
    settled     {item: (xyz, quat_xyzw)}  poses after settling in Gazebo
    basket_xy   [x, y]
    held        the item the robot holds in the initial state, or None
    init, goal  PDDL facts;  plan  the optimal PDDL plan
    setup, actions  the motion plan (see skills.py for the step format)
"""

import math
import os
import shutil
import tempfile

import numpy as np

from .. import config, objects
from ..gazebo.server import GzServer
from ..gazebo.world import WORLD_NAME, problem_world
from ..planning import pddl
from ..planning.geometry2d import polygon_inside_circle, polygon_inside_rect, polygons_intersect
from ..planning.motion import PlanError
from ..planning.placement import Placement, placement_at, placement_pose
from ..planning.skills import Skills
from ..transforms import inv_tf, quat_to_matrix

# Settling check: an item that moves or tilts more than this is not stable
# where it was put.
STABLE_MOVE = {"table": 0.004, "basket": 0.02}   # m
STABLE_TILT = 2.0        # deg
SETTLE_TIME = 2.0        # s of simulated time


class SurveyViews:
    """Pinhole model of the wrist camera at the survey views of
    config/perception.yaml: what the real robot sees before every action."""

    def __init__(self, cell):
        import yaml

        from ..autonomy.views import view_from_config
        with open(config.path("config", "perception.yaml")) as f:
            views = yaml.safe_load(f)["survey_views"]
        wc = cell["wrist_camera"]
        self.W, self.H = wc["width"], wc["height"]
        self.f = (self.W / 2) / math.tan(wc["horizontal_fov"] / 2)
        self.T_cam_world = [inv_tf(view_from_config(v)) for v in views]

    def visible(self, p, margin=10):
        """True if all points are inside the image of one of the survey views."""
        p = np.atleast_2d(p)
        for T in self.T_cam_world:
            c = p @ T[:3, :3].T + T[:3, 3]
            z = c[:, 2]
            if np.any(z <= 0.05):
                continue
            u = self.W / 2 + self.f * c[:, 0] / z
            v = self.H / 2 + self.f * c[:, 1] / z
            if np.all((u > margin) & (u < self.W - margin) & (v > margin) & (v < self.H - margin)):
                return True
        return False


class GenerationError(RuntimeError):
    pass


class NoAcceptedInit(GenerationError):
    """No sampled initial state passed make_layout's `accept`."""


def _area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


class Generator:
    """Samples problems in the current cell (cell.yaml, items.yaml) and plans
    their motions. `seed` makes everything reproducible."""

    def __init__(self, seed=None, randomize_basket=False, log=print):
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.cell = config.load_cell()
        self.items = objects.load_items()
        self.basket = objects.load_basket(self.cell)
        self.cam = SurveyViews(self.cell)
        self.basket_xy = self.sample_basket_xy() if randomize_basket else np.array(self.cell["basket"]["center_xy"], float)
        self.skills = Skills(self.cell, self.items, self.basket, basket_xy=self.basket_xy)
        self.log = log
        self.home = np.array(self.cell["robot"]["home_joints"], float)

    def sample_basket_xy(self, tries=500):
        """A random basket position on the board (cell.yaml basket.random_x/y):
        the rim on the table top, in the survey view and clear of the robot base."""
        b, t = self.cell["basket"], self.cell["table"]
        r = self.basket.r_top
        a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        for _ in range(tries):
            x, y = self.rng.uniform(*b["random_x"]), self.rng.uniform(*b["random_y"])
            if math.hypot(x, y) - r < b["min_base_distance"]:
                continue
            if x - r < t["x_min"] or x + r > t["x_max"] or y - r < t["y_min"] or y + r > t["y_max"]:
                continue
            rim = np.c_[x + r * np.cos(a), y + r * np.sin(a), np.full(len(a), self.basket.height)]
            if self.cam.visible(rim, margin=5):
                return np.array([x, y])
        raise GenerationError("no basket position found")

    # ---- table placement ----------------------------------------------------------

    def table_region(self):
        pl = self.cell["placement"]
        t = self.cell["table"]
        m = pl["edge_margin"]
        x0 = max(pl["x_range"][0], t["x_min"] + m)
        x1 = min(pl["x_range"][1], t["x_max"] - m)
        y0 = max(pl["y_range"][0], t["y_min"] + m)
        y1 = min(pl["y_range"][1], t["y_max"] - m)
        keep = [(0.0, 0.0, pl["robot_keepout_radius"]),
                (self.basket_xy[0], self.basket_xy[1], self.basket.r_top + pl["basket_margin"])]
        return dict(x_range=(x0, x1), y_range=(y0, y1), keepout=keep)

    def _table_ok(self, item, p, placed):
        pl = self.cell["placement"]
        reg = self.table_region()
        obj = self.items[item]
        fp = obj.footprint_world(p.rest, p.x, p.y, p.yaw)
        if not polygon_inside_rect(fp, *reg["x_range"], *reg["y_range"]):
            return False
        for cx, cy, r in reg["keepout"]:
            if np.any(np.hypot(fp[:, 0] - cx, fp[:, 1] - cy) < r):
                return False
        rr = np.hypot(fp[:, 0], fp[:, 1])
        if rr.max() > pl["max_reach"] or rr.min() < pl["min_reach"]:
            return False
        top = obj.poses[p.rest].top
        corners = np.vstack([np.c_[fp, np.zeros(len(fp))], np.c_[fp, np.full(len(fp), top)]])
        if not self.cam.visible(corners, margin=15):
            return False
        for q in placed:
            if polygons_intersect(fp, self.items[q.item].footprint_world(q.rest, q.x, q.y, q.yaw),
                                  pl["object_clearance"]):
                return False
        return True

    def sample_table(self, item, placed, tries=1500):
        reg = self.table_region()
        obj = self.items[item]
        rests = list(obj.spec["stable_poses"])
        for _ in range(tries):
            rest = rests[self.rng.integers(len(rests))]
            p = Placement(item, rest, float(self.rng.uniform(*reg["x_range"])),
                          float(self.rng.uniform(*reg["y_range"])), float(self.rng.uniform(-math.pi, math.pi)))
            if self._table_ok(item, p, placed):
                return p
        return None

    # ---- basket placement --------------------------------------------------------------

    def sample_basket(self, item, placed, tries=300):
        """Resting place on the basket floor (no stacking for initial states)."""
        obj = self.items[item]
        bx, by = self.basket_xy
        r_floor = self.basket.inner_radius(0.0) - 0.008
        rests = list(obj.spec["stable_poses"])
        for _ in range(tries):
            rest = rests[self.rng.integers(len(rests))]
            r = r_floor * math.sqrt(self.rng.uniform())
            a = self.rng.uniform(-math.pi, math.pi)
            p = Placement(item, rest, float(bx + r * math.cos(a)), float(by + r * math.sin(a)),
                          float(self.rng.uniform(-math.pi, math.pi)), support="basket",
                          z_support=float(self.skills.basket_floor()))
            fp = obj.footprint_world(rest, p.x, p.y, p.yaw)
            top = obj.poses[rest].top
            r_top = self.basket.inner_radius(min(top, self.basket.height)) - 0.004
            if not polygon_inside_circle(fp, bx, by, min(r_floor, r_top)):
                continue
            if any(polygons_intersect(fp, self.items[q.item].footprint_world(q.rest, q.x, q.y, q.yaw), 0.006)
                   for q in placed):
                continue
            return p
        return None

    # ---- physics ------------------------------------------------------------------------

    def _simulate(self, placements=None, poses=None, t=SETTLE_TIME):
        work = tempfile.mkdtemp(prefix="labgen_")
        try:
            wf = os.path.join(work, "check.sdf")
            with open(wf, "w") as f:
                f.write(problem_world(self.cell, self.items, placements=placements, poses=poses,
                                      basket_xy=self.basket_xy, gui=False, sensors=False, rtf=0, lift=0.0008))
            with GzServer(wf, WORLD_NAME) as gz:
                gz.wait_until(t, timeout=180)
                return gz.poses()
        finally:
            shutil.rmtree(work, ignore_errors=True)

    def settle(self, placements):
        """Settle the arrangement in Gazebo and check that it stays put.

        Returns ({item: (xyz, quat)}, [what is not stable, one line per item])."""
        ps = self._simulate(placements=placements)
        bad = []
        settled = {}
        for p in placements:
            item = self.items[p.item]
            if item.model not in ps:
                bad.append(f"{p.item} (missing from the simulation)")
                continue
            xyz, q = ps[item.model]
            x0, R0 = placement_pose(item, p)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((quat_to_matrix(q) @ R0.T)[2, 2])))))
            move = float(np.linalg.norm(xyz - x0))
            if move > STABLE_MOVE[p.support] or tilt > STABLE_TILT:
                bad.append(f"{p.item} moved {move * 1000:.0f} mm, tilted {tilt:.1f} deg")
            settled[p.item] = (xyz, q)
        if bad:
            return settled, bad
        # Second run from the settled poses: nothing may move any more.
        ps2 = self._simulate(poses=settled)
        for name, (xyz, q) in settled.items():
            xyz2, _ = ps2[self.items[name].model]
            if np.linalg.norm(xyz2 - xyz) > 0.002:
                bad.append(f"{name} still moves after settling")
        return settled, bad

    # ---- layouts ------------------------------------------------------------------------

    def sample_layout(self, n_basket, holding):
        items = list(pddl.ITEMS)
        self.rng.shuffle(items)
        in_basket = items[:n_basket]
        on_table = items[n_basket:]
        placed = []
        for it in list(in_basket):
            p = self.sample_basket(it, placed)
            if p is None:
                in_basket.remove(it)
                on_table.append(it)
            else:
                placed.append(p)
        # Largest footprints first: random sequential packing fails less.
        on_table.sort(key=lambda it: -max(_area(self.items[it].poses[r].footprint)
                                         for r in self.items[it].spec["stable_poses"]))
        for it in on_table:
            p = self.sample_table(it, [q for q in placed if q.support == "table"])
            if p is None:
                raise GenerationError(f"no room on the table for {it}")
            placed.append(p)
        held_item = None
        if holding:
            cands = [p.item for p in placed if p.support == "table"]
            held_item = cands[self.rng.integers(len(cands))]
        return placed, held_item

    def graspable(self, placements):
        """Items on the table that cannot be picked from the home pose."""
        world = {p.item: p for p in placements}
        bad = []
        for p in placements:
            if p.support != "table":
                continue
            try:
                self.skills.plan_pick(self.home, p.item, world)
            except PlanError:
                bad.append(p.item)
        return bad

    def sample_accepted_layout(self, n_basket, holding, accept, tries=30):
        """sample_layout until accept(initial state) holds (e.g. the goal is
        reachable from it). Cheap: nothing is settled or motion-planned yet."""
        for _ in range(tries):
            placed, held_item = self.sample_layout(n_basket, holding)
            if accept(self.init_state(placed, held_item)):
                return placed, held_item
        raise NoAcceptedInit(f"none of {tries} sampled initial states was accepted")

    def make_layout(self, n_basket, holding, attempts=12, accept=None):
        """A stable, graspable layout. `accept(init)`, if given, filters the
        initial states (PDDL facts) before the physics checks."""
        for k in range(attempts):
            try:
                if accept is None:
                    placed, held_item = self.sample_layout(n_basket, holding)
                else:
                    # A rejection here is not bad luck with the packing:
                    # retrying the same distribution is pointless.
                    placed, held_item = self.sample_accepted_layout(n_basket, holding, accept)
            except NoAcceptedInit:
                raise
            except GenerationError as e:
                self.log(f"  layout attempt {k + 1}: {e}")
                continue
            # Replace items the robot cannot reach or grasp.
            for _ in range(6):
                bad = self.graspable(placed)
                if not bad:
                    break
                self.log(f"  re-placing ungraspable items: {', '.join(bad)}")
                for it in bad:
                    others = [q for q in placed if q.item != it and q.support == "table"]
                    p = self.sample_table(it, others)
                    if p is not None:
                        placed = [q for q in placed if q.item != it] + [p]
            else:
                continue
            if self.graspable(placed):
                continue
            self.log("  checking stability in Gazebo ...")
            settled, unstable = self.settle(placed)
            if unstable:
                self.log(f"  layout attempt {k + 1}: unstable {', '.join(unstable)}")
                continue
            return placed, settled, held_item
        raise GenerationError("could not produce a stable layout")

    # ---- goals ----------------------------------------------------------------------------

    def init_state(self, placements, held_item):
        s = set()
        for p in placements:
            if p.item == held_item:
                s.add(("robot-gripping", p.item))
            else:
                s.add(("on-table" if p.support == "table" else "in-basket", p.item))
        if held_item is None:
            s.add(pddl.EMPTY)
        return frozenset(s)

    def sample_goal(self, init, placements, held_item, min_a, max_a, target=None):
        """A goal whose optimal plan has `target` actions (or any length in
        [min_a, max_a] when target is None).

        Moving an item into the basket costs two actions (pick-up,
        put-in-basket); ending with an item in the gripper costs one more; an
        item held at the start costs one action to put down first."""
        on_table = [f[1] for f in init if f[0] == "on-table"]
        for _ in range(300):
            L = target if target is not None else int(self.rng.integers(min_a, max_a + 1))
            goal = set()
            if held_item:
                # Put the held item away first (1 action), or carry it to the
                # basket as one of the moves.
                choice = self.rng.uniform()
                if choice < 0.4:
                    goal.add(("in-basket", held_item))
                    budget = L - 1
                elif choice < 0.7:
                    goal.add(("on-table", held_item))
                    budget = L - 1
                else:
                    budget = L  # it is put down as part of the plan anyway
            else:
                budget = L
            final_hold = budget % 2 == 1
            k = (budget - (1 if final_hold else 0)) // 2
            pool = [i for i in on_table]
            if k + (1 if final_hold else 0) > len(pool) or k < 0:
                continue
            chosen = list(self.rng.choice(pool, size=k + (1 if final_hold else 0), replace=False))
            for i in chosen[:k]:
                goal.add(("in-basket", i))
            if final_hold:
                goal.add(("robot-gripping", chosen[k]))
            elif self.rng.uniform() < 0.4:
                goal.add(pddl.EMPTY)
            unchanged = [i for i in on_table if i not in chosen]
            for i in self.rng.permutation(unchanged)[: int(self.rng.integers(0, 3))]:
                goal.add(("on-table", i))
            plan = pddl.plan_bfs(init, goal)
            if plan is not None and min_a <= len(plan) <= max_a and (target is None or len(plan) == target):
                return frozenset(goal), plan
        raise GenerationError("no goal with a suitable plan length")

    def plan_motions(self, placements, settled, held_item, plan):
        """Motion-plan the setup (grasping the initially held item) and every
        PDDL action. Raises PlanError if any step is infeasible."""
        sk = self.skills
        world = {p.item: placement_at(self.items[p.item], p, *settled[p.item]) for p in placements}
        q = self.home.copy()
        setup = []
        held = None
        if held_item is not None:
            steps, q, held = sk.plan_pick(q, held_item, world)
            del world[held_item]
            home_steps, q = sk.plan_home(q, world, held=held)
            setup = steps + home_steps
        actions = []
        reg = self.table_region()
        for act in plan:
            name, item = act.strip("()").split()
            if name == "pick-up":
                steps, q, held = sk.plan_pick(q, item, world)
                del world[item]
                info = {}
            elif name == "put-in-basket":
                tgt, steps, q = sk.find_spot(q, held, "basket", world, prefer=tuple(self.basket_xy),
                                             rng=self.rng)
                world[item] = tgt
                held = None
                info = {"target": tgt.as_dict()}
            else:
                old = next(p for p in placements if p.item == item)
                tgt, steps, q = sk.find_spot(q, held, "table", world, region=reg, prefer=(old.x, old.y))
                world[item] = tgt
                held = None
                info = {"target": tgt.as_dict()}
            actions.append({"action": act, "steps": steps, **info})
        home_steps, q = sk.plan_home(q, world, held=held)
        actions.append({"action": "(return home)", "steps": home_steps})
        return setup, actions

    # ---- top level -------------------------------------------------------------------------

    def generate(self, n_basket=None, p_holding=0.3, min_actions=2, max_actions=10, goal_tries=8, target=None):
        # Draw the plan length first (uniform), so long plans are as common
        # as short ones; long plans need room in the basket.
        if target is None:
            target = int(self.rng.integers(min_actions, max_actions + 1))
        if n_basket is None:
            room = max(0, 4 - max(0, target - 4) // 2)
            n_basket = int(self.rng.integers(0, room + 1))
        holding = bool(self.rng.uniform() < p_holding)
        self.log(f"  target plan length {target}, {n_basket} items in the basket, "
                 f"{'holding an item' if holding else 'gripper empty'} at the start")
        for attempt in range(4):
            placements, settled, held_item = self.make_layout(n_basket, holding)
            placements = [placement_at(self.items[p.item], p, *settled[p.item]) for p in placements]
            init = self.init_state(placements, held_item)
            for g in range(goal_tries):
                # Keep the drawn length for most tries, then accept any.
                tgt = target if g < goal_tries - 2 else None
                goal, plan = self.sample_goal(init, placements, held_item, min_actions, max_actions, tgt)
                try:
                    setup, actions = self.plan_motions(placements, settled, held_item, plan)
                except PlanError as e:
                    self.log(f"  goal {g + 1} rejected ({len(plan)} actions): {e}")
                    continue
                return dict(placements=placements, settled=settled, held=held_item, init=init, goal=goal,
                            plan=plan, setup=setup, actions=actions, basket_xy=self.basket_xy.tolist())
            self.log(f"  no feasible goal for layout {attempt + 1}, sampling a new layout")
        raise GenerationError("could not find a feasible goal")
