#!/usr/bin/env python3
"""Type a goal; the robot plans the actions (UPF + Fast Downward) and performs them.

    run/4_goals.sh sim|real                      (the usual way: sets up the environment)
    ros2 run lab_replica goal_console.py [--sim] [--speed-scale S] [--step]

Needs the robot (run/1_robot.sh) and the wrist camera (run/2_camera.sh). Loads
SAM 3 and, once the arm, the gripper and the camera answer, shows the state,
how to write a goal, and a prompt.

The state starts as: every item on the table, the gripper empty. After each
goal it is the state the performed actions should have produced (nothing
perceives facts such as in-basket); `reset` sets it back to the start. A
goal is planned from that state, and the plan's actions are performed one by
one as in the action console, stopping at the first that fails.

Ctrl-C during an action stops the arm; Ctrl-C or Ctrl-D at the prompt quits.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica.console import PREDICATES, Robot, Terminal, banner, common_arguments, wrap_words  # noqa: E402
from lab_replica.planning import pddl  # noqa: E402
from lab_replica.tamp import planner, task  # noqa: E402
from lab_replica.tamp.execute import effect  # noqa: E402
from lab_replica.tamp.goal import CONNECTIVES, GoalError, holds, parse_goal, to_text  # noqa: E402

START = task.most_permissive_init()      # every item on the table, gripper empty


def describe(state):
    table = [i for i in pddl.ITEMS if ("on-table", i) in state]
    basket = [i for i in pddl.ITEMS if ("in-basket", i) in state]
    held = pddl.held(state)
    return table, basket, held


def show_state(term, state):
    table, basket, held = describe(state)
    term.say(term.bold("State:   ") + "on the table: " + (wrap_words(table, 23) if table else "nothing"))
    term.say("         in the basket: " + (", ".join(basket) if basket else "nothing"))
    term.say("         gripper: " + (f"holding {held}" if held else "empty"))


def hints(term, state):
    term.say()
    show_state(term, state)
    term.say(term.bold("Goal:    ") + "facts that must all hold at the end, for example")
    term.say("           (in-basket green-mug)")
    term.say("           (in-basket red-can) (robot-gripping green-mug)")
    term.say("           (and (in-basket red-block) (or (in-basket blue-block) (on-table green-block)))")
    facts = [f"({p})" if p == "robot-gripper-empty" else f"({p} <item>)" for p in PREDICATES]
    term.say(term.bold("Facts:   ") + "  ".join(facts))
    term.say("         combined with " + ", ".join(CONNECTIVES) + "  (e.g. (not (on-table red-can)))")
    term.say(term.bold("Items:   ") + wrap_words(pddl.ITEMS, 9))
    term.say(term.dim("Commands: state (show the state), reset (all items on the table, gripper empty), quit. "
                      "Tab completes names."))


def actions(n):
    return f"{n} action" + ("" if n == 1 else "s")


def plan_for(term, state, formula):
    """The planner's plan, or None (said why)."""
    if holds(formula, state):
        term.say("the goal already holds: nothing to do")
        return None
    term.say("planning ...")
    try:
        return task.solve(state, formula, log=lambda s: None)
    except planner.Unsolvable:
        parts = formula[1:] if formula[0] == "and" else [formula]
        bad = [to_text(x) for x in parts if not task.reachable(state, x)]
        term.say(term.colour("the goal cannot be reached from this state", False)
                 + (f": {' '.join(bad)} can never hold" if bad else "")
                 + " (items in the basket never leave it; the gripper holds one item at most)")
    except planner.PlannerError as e:
        term.say(term.colour("planner error", False) + f": {e}")
    return None


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = common_arguments(argparse.ArgumentParser(description=__doc__,
                                                  formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--step", action="store_true", help="wait for Enter before every action of a plan")
    args, _ = ap.parse_known_args()
    words = list(PREDICATES) + list(CONNECTIVES) + list(pddl.ITEMS) + ["state", "reset", "quit"]
    term = Terminal(words, "goals")
    banner(term, "goal console", args.sim)
    try:
        planner.check()
    except planner.PlannerError as e:
        return f"{e}"
    robot = Robot(term, sim=args.sim, speed_scale=args.speed_scale, verbose=args.verbose,
                  return_home=not args.no_home, save_frames=args.save_frames)
    state = START
    if robot.holding:
        term.say(term.colour("warning", False) + ": the gripper holds something, but the goal console starts "
                 "from every item on the table and the gripper empty. Take the item out of the gripper "
                 "(or run (put-on-table <item>) in the action console), then restart.")
    else:
        robot.ex.set_held(None)
    term.say(f"Ready. Log and camera views: {robot.run_dir}")
    try:
        while True:
            hints(term, state)
            line = term.ask("goal> ")
            if line is None or line.strip().lower() in ("quit", "exit", "q"):
                break
            cmd = line.strip().lower()
            if not cmd:
                continue
            if cmd == "state":
                continue            # the hints show it
            if cmd == "reset":
                state = START
                term.say("state reset: every item on the table, the gripper empty")
                if robot.grip():
                    term.say(term.colour("warning", False) + ": the gripper holds something")
                continue
            try:
                formula, _ = parse_goal(line)
            except (GoalError, OSError) as e:
                # The hints list the items and predicates already.
                why = str(e).split("; the items are")[0].split("; the predicates are")[0]
                term.say(term.colour("invalid goal", False) + f": {why}")
                continue
            term.say(f"goal: {to_text(formula)}")
            plan = plan_for(term, state, formula)
            if not plan:
                continue
            term.say(f"plan ({actions(len(plan))}):")
            for k, a in enumerate(plan):
                term.say(f"  {k + 1}. {a}")
            for k, action in enumerate(plan):
                nxt = effect(state, action)
                if args.step:
                    answer = term.ask(f"Enter performs action {k + 1}/{len(plan)} {action} (n + Enter stops) ")
                    if answer is None or answer.strip().lower().startswith("n"):
                        term.say("stopped before the action")
                        break
                term.say(f"performing action {k + 1}/{len(plan)}: {action}")
                ok, msg = robot.execute(action)
                if not ok:
                    term.say(term.colour("action failed", False) + f": {msg}")
                    term.say(term.colour("goal not reached", False)
                             + f": stopped at action {k + 1} of {len(plan)}; the state below is the one "
                               "before that action")
                    grip, held = robot.grip(), pddl.held(state)
                    if grip is not None and grip != (held is not None):
                        term.say(term.colour("warning", False) + ": the gripper is "
                                 + ("holding something" if grip else "empty") + ", unlike the state below: "
                                 "check the scene, then use `reset` or the action console")
                    break
                term.say(term.colour("action success", True) + f": {msg}")
                state = nxt
            else:
                term.say(term.colour("goal reached", True)
                         + f" ({actions(len(plan))}; the state is what they should have produced)")
    except KeyboardInterrupt:
        term.say()
    finally:
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
