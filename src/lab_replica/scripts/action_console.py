#!/usr/bin/env python3
"""Type a PDDL action, and the robot performs it autonomously.

    run/3_actions.sh sim|real                    (the usual way: sets up the environment)
    ros2 run lab_replica action_console.py [--sim] [--speed-scale S]

Needs the robot (run/1_robot.sh) and the wrist camera (run/2_camera.sh). Loads
SAM 3 and, once the arm, the gripper and the camera answer, shows the
actions, the items and a prompt. Each action is perceived, planned and
executed by the skills (README section A.3); the answer is

    invalid action string: <why>     nothing was sent to the robot
    performing action                then, when it is over:
    action success: <what was done>
    action failed: <reason>          a refused action leaves the scene as it was

A window shows the wrist camera live (colour and depth side by side) and
every view the robot takes, annotated (viewer.py); --no-window leaves it out.

Ctrl-C during an action stops the arm; Ctrl-C or Ctrl-D at the prompt quits.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica.console import (ACTIONS, InvalidAction, Robot, Terminal, banner, common_arguments,  # noqa: E402
                                 parse_action, wrap_words)
from lab_replica.planning import pddl  # noqa: E402


def hints(term):
    term.say()
    term.say(term.bold("Actions: ") + "   ".join(f"({a} <item>)" for a in ACTIONS))
    term.say(term.bold("Items:   ") + wrap_words(pddl.ITEMS, 9))
    term.say(term.bold("Example: ") + "(pick-up green-mug)   then   (put-in-basket green-mug)")
    term.say(term.dim("Tab completes names; Ctrl-D quits."))


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = common_arguments(argparse.ArgumentParser(description=__doc__,
                                                  formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--no-window", action="store_true",
                    help="do not open the camera window (live colour and depth, annotated observations)")
    args, _ = ap.parse_known_args()
    term = Terminal(list(ACTIONS) + list(pddl.ITEMS), "actions")
    banner(term, "action console", args.sim)
    robot = Robot(term, sim=args.sim, speed_scale=args.speed_scale, verbose=args.verbose,
                  return_home=not args.no_home, save_frames=args.save_frames, window=not args.no_window)
    term.say(f"Ready. Log and camera views: {robot.run_dir}")
    try:
        while True:
            hints(term)
            with robot.idle():
                line = term.ask("action> ")
            if line is None or line.strip().lower() in ("quit", "exit", "q"):
                break
            if not line.strip():
                continue
            try:
                action = parse_action(line)
            except InvalidAction as e:
                term.say(term.colour("invalid action string", False) + f": {e}")
                continue
            term.say(f"performing action {action}")
            ok, msg = robot.execute(action)
            term.say(term.colour("action success", True) + f": {msg}" if ok else
                     term.colour("action failed", False) + f": {msg}")
    except KeyboardInterrupt:
        term.say()
    finally:
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
