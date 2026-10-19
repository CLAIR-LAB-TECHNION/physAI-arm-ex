"""Bring a problem up to date with the measurements (cell.yaml, items.yaml,
the problem's layout.yaml) and plan it again. See scripts/replan_problem.py."""

import math
import os

import numpy as np
import yaml

from ..gazebo.build import build_models
from ..planning import pddl
from ..planning.motion import PlanError
from ..planning.placement import Placement, placement_at, placement_pose
from ..transforms import matrix_to_quat
from . import files
from .generator import Generator


class ReplanError(RuntimeError):
    pass


def measured_placements(gen, layout, log=print):
    """Placements from layout.yaml with the current item sizes and basket position.

    x, y and yaw_deg of every item come from layout.yaml; the height follows
    from its resting pose and support. Items in the basket move with the
    basket when basket.center_xy in cell.yaml differs from layout.yaml."""
    shift = gen.basket_xy - np.array(layout["basket"]["center_xy"], float)
    out = []
    for item, o in layout["objects"].items():
        x, y = o["xyz"][:2]
        if o["support"] == "basket":
            x, y = x + shift[0], y + shift[1]
            z_support = gen.skills.basket_floor()
        else:
            z_support = gen.cell["table"]["surface_z"]
        out.append(Placement(item, o["resting_pose"], float(x), float(y), math.radians(o["yaw_deg"]),
                             o["support"], float(z_support)))
    if np.abs(shift).max() > 1e-4:
        n = sum(p.support == "basket" for p in out)
        log(f"  basket at {list(np.round(gen.basket_xy, 4))} (layout.yaml: {layout['basket']['center_xy']})"
            + (f"; the {n} item(s) in it move with it" if n else ""))
    return out


def replan(problem_dir, settle=True, rebuild_models=True, render=False, log=print):
    """Rebuild the models, settle the layout, plan, and rewrite the problem's
    world.sdf, plan.json, layout.png, solution.py and README.md. Nothing is
    written if no collision-free plan exists (ReplanError)."""
    with open(os.path.join(problem_dir, "layout.yaml")) as f:
        layout = yaml.safe_load(f)
    init, goal = pddl.read_problem(os.path.join(problem_dir, "problem.pddl"))
    if rebuild_models:
        log("rebuilding the Gazebo models from cell.yaml and items.yaml")
        build_models(log=lambda *a: None)

    gen = Generator(seed=layout.get("seed"), log=log)   # basket from cell.yaml
    placements = measured_placements(gen, layout, log)
    if settle:
        log("settling the arrangement in Gazebo ...")
        settled, bad = gen.settle(placements)
        if bad:
            log("WARNING: not stable where layout.yaml puts them (overlapping items, an item off its "
                "support, or a wrong resting pose):\n    " + "\n    ".join(bad))
        else:
            log("  every item rests where layout.yaml puts it")
    else:
        settled = {}
        for p in placements:
            xyz, R = placement_pose(gen.items[p.item], p)
            settled[p.item] = (xyz, np.array(matrix_to_quat(R)))

    placements = [placement_at(gen.items[p.item], p, *settled[p.item]) for p in placements]
    plan = pddl.plan_bfs(init, goal)
    log("plan: " + " ".join(plan))
    held = layout.get("held_at_start")
    try:
        setup, actions = gen.plan_motions(placements, settled, held, plan)
    except PlanError as e:
        raise ReplanError(f"no collision-free motion for the measured scene: {e}") from e

    prob = dict(placements=placements, settled=settled, basket_xy=gen.basket_xy.tolist(), held=held,
                init=init, goal=goal, plan=plan, setup=setup, actions=actions)
    files.write_scene_and_plan(problem_dir, prob, layout["name"], gen.cell, gen.items, render=render, log=log)
    log(f"wrote world.sdf, plan.json, layout.png, solution.py, README.md{', render.png' if render else ''} "
        f"in {problem_dir}")
    return prob
