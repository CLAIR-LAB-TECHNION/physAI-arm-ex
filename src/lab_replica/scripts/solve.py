#!/usr/bin/env python3
"""Solve a goal in the lab cell: Fast Downward plans, the autonomous skills execute.

    ros2 run lab_replica solve.py GOAL [scene] [options]
    ros2 run lab_replica solve.py --task NAME [options]        # run an existing task again

GOAL is a PDDL goal over the fixed domain (config/domain.pddl) and the twelve
items, or a .pddl problem file whose (:goal ...) is taken:

    "(in-basket green-mug)"
    "(in-basket green-mug) (robot-gripping red-can)"           (a conjunction)
    "(and (in-basket roll-of-blue-tape) (not (on-table red-block)))"
    my_problem.pddl

The domain and the objects are always the same; the initial state comes from
the scene:

    (default)              a new random scene, sampled until the goal is
                           reachable from it (--seed, --basket-items, --p-holding)
    --scene PROBLEM        the scene of an existing problem or task directory
                           (problems/, problems_gen/, problems_tamp/ or a path)

Then:
  1. the task is written to problems_tamp/<name>/ (problem.pddl, world.sdf,
     layout, README.md with the table set-up);
  2. Fast Downward plans (plan.txt);
  3. unless --plan-only: the simulation (sim.launch.py with the scene) and the
     skill server (skill_server.py --sim) are started, every action of the
     plan is executed through /lab_skills/execute, and the goal is checked
     against the simulator's ground truth. With --attach the simulation or
     the real robot and the skill server are expected to be running already.

Exits 0 when the plan was executed and (in simulation) the goal holds.
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from lab_replica import config  # noqa: E402
from lab_replica.planning import pddl  # noqa: E402
from lab_replica.tamp import fast_downward, task  # noqa: E402
from lab_replica.tamp.goal import GoalError, holds, parse_goal, to_text, unsatisfied_facts  # noqa: E402

HELD_FILE = os.path.expanduser("~/.ros/lab_skills_held.json")


# ---- processes ---------------------------------------------------------------------------

def start(cmd, env, log, ready, timeout=300):
    f = open(log, "w")
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if p.poll() is not None:
            raise RuntimeError(f"{' '.join(cmd[:3])} exited, see {log}")
        if ready in open(log, errors="ignore").read():
            return p
        time.sleep(1)
    stop(p)
    raise RuntimeError(f"{' '.join(cmd[:3])} not ready after {timeout} s, see {log}")


def stop(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=20)
    except Exception:  # noqa: BLE001
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()


def venv_python():
    py = os.path.join(config.workspace_root(), ".venv", "bin", "python")
    return py if os.path.exists(py) else sys.executable


# ---- the task ------------------------------------------------------------------------------

def make_task(args):
    """(task dir, init, goal formula, plan); writes the task directory."""
    if args.task:
        d = task.task_dir(args.task)
        init, formula, _ = task.read_task(d)
        print(f"task {os.path.relpath(d, config.workspace_root())}")
        print(f"  goal: {to_text(formula)}")
        plan = task.solve(init, formula, out_dir=d, log=print, time_limit=args.time_limit)
        return d, init, formula, plan

    try:
        formula, source = parse_goal(args.goal)
    except (GoalError, OSError) as e:
        sys.exit(f"bad goal: {e}")
    print(f"goal ({source}): {to_text(formula)}")
    if not task.reachable(task.most_permissive_init(), formula):
        sys.exit("the goal can never hold (it contradicts the domain, e.g. two items in the gripper)")

    name = args.name or task.new_name()
    d = os.path.join(task.tasks_root(), name)
    if os.path.exists(d):
        sys.exit(f"{d} exists: choose another --name, or run it with --task {name}")
    try:
        if args.scene:
            src = task.scene_dir(args.scene)
            init, _, _ = task.copy_scene(src, d)
            print(f"scene of {os.path.relpath(src, config.workspace_root())}: {task.describe_init(init)}")
        else:
            from lab_replica.problems.generator import GenerationError
            print("generating a scene (stable layout, checked in Gazebo) ...")
            try:
                init, _, seed = task.generate_scene(
                    formula, d, seed=args.seed, n_basket=args.basket_items, p_holding=args.p_holding,
                    randomize_basket=args.randomize_basket, render=not args.no_render)
            except GenerationError as e:
                raise SystemExit(f"no scene: {e}")
            print(f"scene (seed {seed}): {task.describe_init(init)}")
        try:
            plan = task.solve(init, formula, out_dir=d, time_limit=args.time_limit)
        except fast_downward.Unsolvable:
            parts = formula[1:] if formula[0] == "and" else [formula]
            bad = [to_text(x) for x in parts if not task.reachable(init, x)]
            raise SystemExit("the goal cannot be reached from this scene's initial state"
                             + (f": {' '.join(bad)} can never hold" if bad else "")
                             + " (items in the basket never leave it; the gripper holds one item at most)")
        task.write_task(d, init, formula, plan, source)
    except BaseException:
        shutil.rmtree(d, ignore_errors=True)
        raise
    print(f"task written to {os.path.relpath(d, config.workspace_root())}/")
    return d, init, formula, plan


# ---- main ------------------------------------------------------------------------------------

def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("goal", nargs="?", help="PDDL goal, or a .pddl problem file (its :goal)")
    ap.add_argument("--task", help="run an existing task of problems_tamp/ (name or path) instead of a new goal")
    ap.add_argument("--name", help="task directory name (default task_<date>_<time>)")
    g = ap.add_argument_group("scene (initial state)")
    g.add_argument("--scene", help="take the scene of this problem or task directory")
    g.add_argument("--seed", type=int, help="seed of a generated scene (default: from the clock)")
    g.add_argument("--basket-items", type=int, help="items in the basket at the start (default random 0-3)")
    g.add_argument("--p-holding", type=float, default=0.3, help="probability that the robot starts holding an item")
    g.add_argument("--randomize-basket", action="store_true", help="random basket position on the board (cell.yaml basket.random_x/y)")
    g.add_argument("--no-render", action="store_true", help="skip render.png (the lab camera's view)")
    g = ap.add_argument_group("planning and execution")
    g.add_argument("--time-limit", type=int, default=60, help="Fast Downward's time limit [s]")
    g.add_argument("--plan-only", action="store_true", help="write the task and the plan, do not execute")
    g.add_argument("--attach", action="store_true",
                   help="use the running simulation or real robot and skill server instead of starting them")
    g.add_argument("--headless", action="store_true", help="no Gazebo window")
    g.add_argument("--domain", type=int, help="ROS_DOMAIN_ID for everything started here (default: the current one)")
    g.add_argument("--speed-scale", type=float, default=1.0, help="skill server speed scale (when started here)")
    g.add_argument("--step", action="store_true", help="wait for Enter before every action")
    g.add_argument("--no-setup", action="store_true",
                   help="do not pick up the initially held item first (it is already in the gripper)")
    g.add_argument("--keep-running", action="store_true", help="keep the simulation up afterwards, until Ctrl-C")
    args = ap.parse_args()
    if bool(args.goal) == bool(args.task):
        ap.error("give a GOAL or --task NAME")
    if args.task and args.scene:
        ap.error("--scene is for a new goal; a task keeps its scene")

    d, init, formula, plan = make_task(args)
    print(f"plan ({len(plan)} actions, Fast Downward):")
    for k, a in enumerate(plan):
        print(f"  {k + 1}. {a}")
    if args.plan_only:
        return 0

    if args.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    if not args.attach:
        os.environ.setdefault("IGN_PARTITION", f"lab_tamp_{os.environ.get('ROS_DOMAIN_ID', '0')}")
    env = dict(os.environ)

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    from lab_replica.tamp.execute import SkillClient, execute_plan
    from lab_replica.tamp.sim_truth import SimTruth

    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    sim = server = None
    logs = os.path.join(d, "logs")
    os.makedirs(logs, exist_ok=True)
    result = dict(task=os.path.basename(d), goal=to_text(formula), init=sorted(map(pddl.fact_str, init)),
                  plan=plan, started=time.strftime("%Y-%m-%d %H:%M:%S"), attach=args.attach)
    code = 1
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        if not args.attach:
            if os.path.exists(HELD_FILE):
                os.remove(HELD_FILE)     # the new simulation starts with an empty gripper
            print("starting the simulation ...")
            sim = start(["ros2", "launch", "lab_replica", "sim.launch.py", f"problem:={d}",
                         f"gui:={'false' if args.headless else 'true'}"], env, f"{logs}/sim.log", "RG6 driver ready")
            print("starting the skill server (loads SAM 3) ...")
            server = start([venv_python(), os.path.join(HERE, "skill_server.py"), "--sim",
                            "--speed-scale", str(args.speed_scale),
                            "--run-dir", os.path.join(config.workspace_root(), "runs", os.path.basename(d))],
                           env, f"{logs}/skill_server.log", "ready: /lab_skills/execute")
        node = Node("solve")
        skills = SkillClient(node)
        truth = SimTruth(node)
        if not skills.wait(30.0):
            raise RuntimeError("no /lab_skills/execute: start skill_server.py (or run without --attach)")
        sim_truth = truth.available(10.0)
        if sim_truth:
            before = truth.facts()
            expected = set(init)
            held = pddl.held(init)
            if held and not args.no_setup:
                expected = (expected - {("robot-gripping", held)}) | {("on-table", held), pddl.EMPTY}
            if before != expected:
                print("warning: the simulated scene does not match the task's initial state (wrong scene?): "
                      f"differs in {' '.join(map(pddl.fact_str, sorted(before ^ expected)))}")
        t0 = time.time()
        report = execute_plan(skills, init, plan, setup=not args.no_setup, step=args.step)
        result.update(report, seconds=round(time.time() - t0, 1))
        print(f"\n{'all ' + str(len(plan)) + ' actions done' if report['ok'] else 'stopped: an action failed'} "
              f"in {result['seconds']:.0f} s")
        if sim_truth:
            truth.spin(2.0)
            facts = truth.facts()
            ok = holds(formula, facts)
            result.update(goal_holds=ok, true_state=sorted(map(pddl.fact_str, facts)))
            print(f"ground truth (simulator): goal {'HOLDS' if ok else 'does NOT hold'}"
                  + ("" if ok else "; missing " + " ".join(unsatisfied_facts(formula, facts))))
            print("  state: " + " ".join(sorted(map(pddl.fact_str, facts))))
            code = 0 if ok and report["ok"] else 1
        else:
            print("(no simulator ground truth: the goal was not checked)")
            code = 0 if report["ok"] else 1
        with open(os.path.join(d, "result.json"), "w") as f:
            json.dump(result, f, indent=1)
        print(f"report: {os.path.relpath(os.path.join(d, 'result.json'), config.workspace_root())}")
        if args.keep_running and sim is not None:
            print("simulation still running; Ctrl-C to stop it")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\ninterrupted")
        code = 130
    except RuntimeError as e:
        print(f"ERROR: {e}")
    finally:
        stop(server)
        stop(sim)
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
