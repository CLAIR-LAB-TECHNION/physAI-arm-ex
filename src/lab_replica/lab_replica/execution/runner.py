"""Run a hard-coded solution (SETUP + ACTIONS of a problem's solution.py) on
whatever is running: the simulation (sim.launch.py) or the real robot
(real.launch.py). The trajectory controller is asked from the controller
manager, so the command is the same for both.

If the arm is not at the solution's start pose, it first offers a slow,
clearance-checked joint move there.
"""

import argparse
import sys
import time

import numpy as np

START_TOLERANCE = 0.02   # rad: closer than this to the start pose counts as there


def add_args(ap):
    ap.add_argument("--speed-scale", type=float, default=1.0,
                    help="scale all motion speeds (e.g. 0.3 for a first run on the real robot); "
                         "the pendant's speed slider scales on top of it")
    ap.add_argument("--step", action="store_true",
                    help="wait for Enter before the setup and every action")
    ap.add_argument("--step-motions", action="store_true",
                    help="wait for Enter before every single arm motion and gripper command")
    ap.add_argument("--from-action", type=int, default=1, metavar="N",
                    help="start with action N (skips the setup unless N is 1)")
    ap.add_argument("--skip-setup", action="store_true",
                    help="do not run the setup grasp (the item is already in the gripper)")
    ap.add_argument("--dry-run", action="store_true", help="only list the steps and their durations")
    return ap


def run(setup, actions, args, title=""):
    if args.dry_run:
        _print(setup, actions, title)
        return 0
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    from .executor import ExecutionError, Executor, active_trajectory_controller

    # Ctrl-C raises KeyboardInterrupt but leaves the ROS context alive, so the
    # running goal can still be cancelled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ex = None
    try:
        probe = rclpy.create_node("lab_controller_probe")
        try:
            print("asking /controller_manager for the active trajectory controller ...")
            controller = active_trajectory_controller(probe)
        finally:
            probe.destroy_node()
        ex = Executor(controller=controller, speed_scale=args.speed_scale)
        print(f"waiting for /{controller} and /rg6/gripper_command ...")
        ex.wait_ready()

        run_setup = bool(setup) and not args.skip_setup and args.from_action == 1
        todo = ([("setup: grasp the item the robot holds in the initial state", setup)] if run_setup else [])
        todo += [(f"{k}/{len(actions)} {a['action']}", a["steps"])
                 for k, a in enumerate(actions, start=1) if k >= args.from_action]
        first = next(s for _, steps in todo for s in steps if s["kind"] == "traj")
        q_start = np.array(first["points"][0])
        if np.abs(ex.current() - q_start).max() > START_TOLERANCE:
            if go_to_start(ex, q_start) != 0:
                return 2
            input("at the start pose; press Enter to start the solution (Ctrl-C aborts) ")

        confirm = None
        if args.step_motions:
            def confirm(text):
                input(f"    press Enter for {text} (Ctrl-C aborts) ")
        t0 = time.time()
        for name, steps in todo:
            if args.step or args.step_motions:
                input(f"press Enter for {name} (Ctrl-C aborts) ")
            print(f"== {name}")
            ex.run_steps(steps, confirm=confirm)
        print(f"done in {time.time() - t0:.0f} s")
        return 0
    except ExecutionError as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        n = ex.cancel_active() if ex else 0
        print(f"interrupted{' - cancelled the running motion' if n else ''}", file=sys.stderr)
        return 130
    finally:
        if ex:
            ex.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# Go-to-start clearance: robot points farther than NEAR_R from the base axis
# must stay above every item on the table (the spray bottle is 0.20 m tall);
# closer in, just above the table.
NEAR_R = 0.22          # m; the shoulder housing reaches 0.20 m out
Z_FAR = 0.25           # m above the table
Z_NEAR = 0.03
JOINT_SPEED = 0.3      # rad/s, before --speed-scale


def go_to_start(ex, q_goal):
    """Straight joint move from where the arm is to q_goal, if no part of the
    arm comes too low on the way; asks for Enter first. Returns 0 when done."""
    from .. import config
    from ..planning.motion import min_jerk
    from ..robot.collision import RobotPoints
    from ..robot.kinematics import robot_urdf

    q_now = ex.current()
    dq = np.abs(q_goal - q_now)
    print(f"The arm is not at the start pose of the solution.\n"
          f"  current joints: {list(np.round(q_now, 4))}\n  start pose:     {list(np.round(q_goal, 4))}")
    rp = RobotPoints(robot_urdf(config.load_cell()["robot"]["ur_type"]))

    def moves(link):
        while link in rp.joints:
            if link == "upper_arm_link":
                return True
            link = rp.joints[link]["parent"]
        return False

    moving = [link for link in rp.points if moves(link)]

    def lowest(q):
        """(lowest point beyond NEAR_R, lowest within it) of the links that move."""
        T = rp.frames(q, 0.09)
        P = np.vstack([rp.points[k] @ T[k][:3, :3].T + T[k][:3, 3] for k in moving])
        far = np.hypot(P[:, 0], P[:, 1]) > NEAR_R
        return P[far, 2].min(initial=np.inf), P[~far, 2].min(initial=np.inf)

    # The start pose itself is safe by construction, even where it is lower
    # than Z_FAR (the open fingertips are 0.23 m above the table there).
    z_far = min(Z_FAR, lowest(q_goal)[0] - 0.005)
    worst, z_low = np.inf, np.inf
    for u in np.linspace(0, 1, 60):
        lo_far, lo_near = lowest(q_now + u * (q_goal - q_now))
        worst = min(worst, lo_far - z_far, lo_near - Z_NEAR)
        z_low = min(z_low, lo_far)
    if worst < 0:
        print(f"REFUSED: a straight joint move there would bring the arm {-worst * 100:.0f} cm too low "
              f"(limit: {z_far:.2f} m above the table beyond {NEAR_R} m from the base axis). "
              f"Move it closer to the start pose with the pendant first.")
        return 2
    T = max(3.0, dq.max() / JOINT_SPEED * 1.875) / ex.speed_scale
    us = np.linspace(0, 1, 21)
    ts = np.interp(us, min_jerk(np.linspace(0, 1, 2001)), np.linspace(0, 1, 2001)) * T * ex.speed_scale
    pts = [q_now + u * (q_goal - q_now) for u in us]
    print(f"Joint move: largest change {np.degrees(dq.max()):.0f} deg, {T:.1f} s; the arm stays "
          f"{z_low:.2f} m or more above the table (beyond {NEAR_R} m from the base axis).")
    input("press Enter to move to the start pose (Ctrl-C aborts) ")
    ex.run_traj(pts, ts, "go to start")
    return 0


def _print(setup, actions, title):
    print(title)
    total = 0.0
    for name, steps in [("setup", setup)] + [(a["action"], a["steps"]) for a in actions]:
        if not steps:
            continue
        d = sum(s["times"][-1] for s in steps if s["kind"] == "traj")
        total += d
        print(f"{name}: {len(steps)} steps, {d:.1f} s of motion")
        for s in steps:
            if s["kind"] == "traj":
                print(f"   arm     {s['label']:18s} {len(s['points']):3d} waypoints  {s['times'][-1]:5.1f} s")
            else:
                print(f"   gripper {s['label']:18s} width {s['width'] * 1000:5.1f} mm  force {s['force']:.0f} N")
    print(f"total motion time {total:.0f} s")


def main_for(setup, actions, doc):
    """Entry point of a solution.py file."""
    ap = add_args(argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter))
    args = ap.parse_args()
    sys.exit(run(setup, actions, args, title=doc.splitlines()[0] if doc else ""))
