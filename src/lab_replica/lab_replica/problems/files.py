"""The files of a problem directory (problems/<name>/).

    README.md     the task, how to run it, where to put the items on the real table
    problem.pddl  initial state and goal;  domain.pddl  the lab's domain (unchanged)
    layout.yaml   where every item stands (the measured placement; edit it
                  and replan when the real table differs)
    layout.png    top view of the layout with coordinates
    world.sdf     the Gazebo world (items at their settled poses)
    plan.json     the motion plan, at full precision
    solution.py   the hard-coded solution: every motion written out
    render.png    the camera view of the initial scene
"""

import json
import math
import os
import shutil
import tempfile

import yaml

from ..gazebo.robot_preview import robot_visual_sdf
from ..gazebo.server import GzServer
from ..gazebo.world import WORLD_NAME, problem_world
from ..planning import pddl
from ..planning.motion import Planner
from ..planning.placement import pose_dict
from ..planning.pick_place import compress_path
from ..robot.kinematics import robot_urdf
from ..transforms import matrix_to_rpy, quat_to_matrix
from .drawing import draw_layout


def write_problem(prob, out_dir, name, cell, items, seed=None, render=True, log=print):
    """Write every file of a freshly generated problem."""
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "domain.pddl"), "w") as f:
        f.write(pddl.domain_text())
    with open(os.path.join(out_dir, "problem.pddl"), "w") as f:
        f.write(pddl.problem_text(prob["init"], prob["goal"]))
    write_layout_yaml(out_dir, prob, name, cell, seed)
    write_scene_and_plan(out_dir, prob, name, cell, items, render=render, log=log)


def write_layout_yaml(out_dir, prob, name, cell, seed=None):
    layout = {
        "name": name,
        "seed": seed,
        "frame": "world: origin on the table surface below the robot base axis, x right (seen from "
                 "the camera), y towards the wall, z up. Robot base_link at "
                 f"{cell['robot']['base_xyz']} with yaw {cell['robot']['base_yaw']}.",
        "basket": {"center_xy": [round(v, 4) for v in prob["basket_xy"]]},
        "held_at_start": prob["held"],
        "objects": {},
        "init": sorted(pddl.fact_str(f) for f in prob["init"]),
        "goal": prob["goal"] if isinstance(prob["goal"], str) else sorted(pddl.fact_str(f) for f in prob["goal"]),
        "optimal_plan": prob["plan"],
    }
    for p in prob["placements"]:
        xyz, q = prob["settled"][p.item]
        R = quat_to_matrix(q)
        layout["objects"][p.item] = {
            "support": p.support,
            "resting_pose": p.rest,
            "xyz": [round(float(v), 4) for v in xyz],
            "rpy_deg": [round(math.degrees(v), 1) for v in matrix_to_rpy(R)],
            "yaw_deg": round(math.degrees(p.yaw), 1),
            **pose_dict(xyz, R),
        }
    with open(os.path.join(out_dir, "layout.yaml"), "w") as f:
        yaml.safe_dump(layout, f, sort_keys=False, width=110)


def write_scene_and_plan(out_dir, prob, name, cell, items, render=False, log=print):
    """world.sdf, plan.json, layout.png, solution.py, README.md (and render.png)."""
    with open(os.path.join(out_dir, "world.sdf"), "w") as f:
        f.write(problem_world(cell, items, poses=prob["settled"], basket_xy=prob["basket_xy"]))
    with open(os.path.join(out_dir, "plan.json"), "w") as f:
        json.dump({"setup": prob["setup"], "actions": prob["actions"], "held_at_start": prob["held"]}, f)
    draw_layout(os.path.join(out_dir, "layout.png"), cell, items, prob, title=name)
    write_solution(out_dir, Planner(cell))
    write_readme(out_dir, cell)
    if render:
        log("  rendering the camera view ...")
        render_camera_view(cell, items, prob, os.path.join(out_dir, "render.png"))


# ---- solution.py -------------------------------------------------------------

MAX_GAP = 0.3       # s, longest time between stored waypoints
PATH_TOL = 0.002    # m, joint interpolation between stored waypoints stays this close to the planned path

SOLUTION_HEADER = '''#!/usr/bin/env python3
"""Hard-coded solution of {name}.

Initial state (PDDL):
{init}
Goal:
{goal}
Plan ({n} actions, optimal):
{plan}

Every arm motion is stored below as joint waypoints [rad] (shoulder_pan,
shoulder_lift, elbow, wrist_1, wrist_2, wrist_3) with time stamps [s]; every
gripper command as a width between the pads [m] and a grip force [N]. The
motions were planned by lab_replica (top-down grasps, collision-checked
against the table, the basket and the other items) for the item poses in
layout.yaml, and are executed open loop.
{setup_note}
Run it (see README.md in this directory):
    ros2 run lab_replica run_solution.py {name} [--speed-scale 0.3] [--step]
The same command drives the simulation and the real robot. Written by
replan_problem.py from plan.json; do not edit by hand.
"""

import os
import sys

'''

SOLUTION_FOOTER = '''

if __name__ == "__main__":
    # The lab_replica sources next to this problem directory.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "src", "lab_replica"))
    from lab_replica.execution.runner import main_for
    main_for(SETUP, ACTIONS, __doc__)
'''


def _format_steps(steps, planner, indent):
    out = []
    pad = " " * indent
    for s in steps:
        if s["kind"] == "gripper":
            extra = f', "drop": {s["drop"]}' if "drop" in s else ""
            out.append(f'{pad}{{"kind": "gripper", "label": "{s["label"]}", "width": {s["width"]:.4f}, '
                       f'"force": {s["force"]:.0f}, "expect_grip": {s["expect_grip"]}{extra}}},')
            continue
        pts, ts = s["points"], s["times"]
        keep = compress_path(planner, pts, tol=PATH_TOL, return_indices=True)
        # The controller fits splines through the waypoints; keep them dense
        # in time so the motion stays close to the checked path.
        dense = [keep[0]]
        for i in keep[1:]:
            for k in range(dense[-1] + 1, i):
                if ts[k] - ts[dense[-1]] >= MAX_GAP:
                    dense.append(k)
            dense.append(i)
        keep = sorted(set(dense))
        out.append(f'{pad}{{"kind": "traj", "label": "{s["label"]}",')
        out.append(f'{pad} "times": [{", ".join(f"{ts[i]:.3f}" for i in keep)}],')
        out.append(f'{pad} "points": [')
        for i in keep:
            out.append(f'{pad}    [{", ".join(f"{v:.5f}" for v in pts[i])}],')
        out.append(f"{pad} ]}},")
    return out


def write_solution(problem_dir, planner):
    """solution.py from plan.json and layout.yaml."""
    with open(os.path.join(problem_dir, "plan.json")) as f:
        plan = json.load(f)
    layout = _read_layout(problem_dir)
    held = plan.get("held_at_start")
    setup_note = ""
    if held:
        setup_note = (f"\nThe initial state has the robot holding {held}. SETUP picks it up from\n"
                      f"its place in layout.yaml first (skip with --skip-setup if it is\n"
                      f"already in the gripper).\n")
    lines = [SOLUTION_HEADER.format(
        name=layout["name"],
        init="\n".join("    " + f for f in layout["init"]),
        goal="\n".join("    " + f for f in layout["goal"]),
        n=len(layout["optimal_plan"]),
        plan="\n".join(f"    {k + 1}. {a}" for k, a in enumerate(layout["optimal_plan"])),
        setup_note=setup_note)]
    lines.append("SETUP = [")
    lines += _format_steps(plan["setup"], planner, 4)
    lines.append("]\n")
    lines.append("ACTIONS = [")
    for a in plan["actions"]:
        lines.append(f'    {{"action": "{a["action"]}",')
        if "target" in a:
            t = a["target"]
            lines.append(f'     # put down at x={t["x"]:.3f} y={t["y"]:.3f} yaw={t["yaw"]:.3f} on the {t["support"]}')
        lines.append('     "steps": [')
        lines += _format_steps(a["steps"], planner, 8)
        lines.append("     ]},")
    lines.append("]")
    out = os.path.join(problem_dir, "solution.py")
    with open(out, "w") as f:
        f.write("\n".join(lines) + SOLUTION_FOOTER)
    os.chmod(out, 0o755)


# ---- README.md ---------------------------------------------------------------

README = """# {name}

<!-- Written by replan_problem.py / generate_problem.py from layout.yaml. Edits are overwritten. -->

{origin}

**Initial state:** {init_text}

**Goal:** {goal}

**Optimal plan ({n} actions):**

{plan}

## Run it

In simulation (two terminals):

```console
ros2 launch lab_replica sim.launch.py problem:={name}
ros2 run lab_replica run_solution.py {name}
ros2 run lab_replica check_goal.py {name}      # afterwards: does the simulated scene satisfy the goal?
```

On the real robot, first set the items up as below, then:

```console
ros2 launch lab_replica real.launch.py         # then press Play on the teach pendant
ros2 run lab_replica run_solution.py {name} --speed-scale 0.3 --step
```

`run_solution.py` first offers to move the arm to the start pose
`{home}` (joint angles in rad) if it is not there.{held_note}

## Setting it up on the real table

Place the items as in [`layout.png`](layout.png) and the table below. The
coordinates are in the world frame: origin on the table surface under the
robot base axis, x to the right as seen from the camera, y towards the wall.
The yaw is the item's rotation about the vertical axis, from +x: for a block
the direction of one face normal, for the mug the direction of its handle,
for the spray bottle the direction of its nozzle. Round items can stand at
any yaw. The basket's centre is at ({bx:.1f}, {by:.1f}) cm.

| # | Item | Where | Resting pose | x [cm] | y [cm] | yaw [deg] |
|---|---|---|---|---|---|---|
{rows}

The grasps tolerate placement errors of about ±1.5 cm (the fingers open
16 mm beyond the item on both sides). If you place an item elsewhere, edit
its `xyz` (x, y) and `yaw_deg` in [`layout.yaml`](layout.yaml) and run
`ros2 run lab_replica replan_problem.py {name}`; check the new plan in
simulation before running it on the robot.
"""


def write_readme(problem_dir, cell):
    layout = _read_layout(problem_dir)
    name = layout["name"]
    held = layout.get("held_at_start")
    rows = []
    for k, (item, o) in enumerate(sorted(layout["objects"].items()), start=1):
        where = "basket" if o["support"] == "basket" else "table"
        if item == held:
            where = "table (the setup step picks it up)"
        rows.append(f"| {k} | {item} | {where} | {o['resting_pose']} | {o['xyz'][0] * 100:.1f} | "
                    f"{o['xyz'][1] * 100:.1f} | {o['yaw_deg']:.0f} |")
    held_note = ""
    if held:
        held_note = (f"\n\nThe initial state has the robot holding the **{held}**. The solution's setup step "
                     f"picks it up from its place in the table below before action 1 (`--skip-setup` if "
                     f"you put it into the gripper yourself).")
    init = [f for f in layout["init"] if f.startswith("(robot") or f.startswith("(in-basket")]
    on_table = sum(f.startswith("(on-table") for f in layout["init"])
    init_text = f"{on_table} items on the table" + "".join(f", {f}" for f in init)
    origin = (f"A problem generated with seed {layout['seed']}." if layout.get("seed") is not None
              else "A problem of the lab replica.")
    bx, by = layout["basket"]["center_xy"]
    text = README.format(
        name=name, origin=origin, init_text=init_text, goal=" ".join(layout["goal"]),
        n=len(layout["optimal_plan"]),
        plan="\n".join(f"{k + 1}. {a}" for k, a in enumerate(layout["optimal_plan"])),
        home=list(cell["robot"]["home_joints"]), held_note=held_note,
        bx=bx * 100, by=by * 100, rows="\n".join(rows))
    with open(os.path.join(problem_dir, "README.md"), "w") as f:
        f.write(text)


def _read_layout(problem_dir):
    with open(os.path.join(problem_dir, "layout.yaml")) as f:
        return yaml.safe_load(f)


# ---- render.png ---------------------------------------------------------------

def render_camera_view(cell, items, prob, out_png):
    """The lab camera's view of the initial scene, with the robot at its start pose."""
    work = tempfile.mkdtemp(prefix="labrender_")
    try:
        frames = os.path.join(work, "frames")
        os.makedirs(frames)
        robot = robot_visual_sdf(robot_urdf(cell["robot"]["ur_type"]), cell["robot"]["home_joints"], 0.1276)
        text = problem_world(cell, items, poses=dict(prob["settled"]), basket_xy=prob["basket_xy"],
                             save_dir=frames, gui=False)
        text = text.replace("  </world>", robot + "\n  </world>")
        wf = os.path.join(work, "render.sdf")
        with open(wf, "w") as f:
            f.write(text)
        with GzServer(wf, WORLD_NAME, rendering=True) as gz:
            gz.wait_until(0.5, timeout=120)
            gz.grab_image(frames, out_png)
    finally:
        shutil.rmtree(work, ignore_errors=True)

