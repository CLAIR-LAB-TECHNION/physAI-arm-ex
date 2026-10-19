#!/usr/bin/env python3
"""Verify the autonomous actions in simulation against the simulator's ground truth.

    python3 tools/verify_actions.py problems_gen/problem_00700 problems_gen/problem_00701 ...
        [--actions 8] [--domain 78] [--report verify.json] [--seed 0]

Run it in the workspace's .venv with the workspace sourced. For each problem
it starts the simulation (sim.launch.py, headless) and the skill server
(skill_server.py --sim), then executes actions one after the other through
/lab_skills/execute, chosen to cover every item and both kinds of put:

    gripper empty    (pick-up X), X an item on the table, the least picked first
    holding X        (put-in-basket X) or (put-on-table X), whichever was done
                     less (the table when the basket holds 5 items or more)

After every action the true state is read from Gazebo (it is never shown to
the robot) and checked:
    effect    the PDDL facts are exactly those the action should produce
    others    no other item moved more than 2 cm or tilted more than 10 deg,
              except items in the basket hit by one dropped into it (they
              are listed as `disturbed`; still a failure if knocked out)
Each action is one line of the report; a summary per action type follows.
"""

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

MOVED = 0.02          # m
TILTED = 10.0         # deg
GRIP_DIST = 0.025     # fingertips within this of an item's box: it is the held one


def start(cmd, env, log, ready, timeout=240):
    f = open(log, "w")
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if p.poll() is not None:
            raise RuntimeError(f"{cmd[0]} exited, see {log}")
        if ready in open(log, errors="ignore").read():
            return p
        time.sleep(2)
    stop(p)
    raise RuntimeError(f"{' '.join(cmd[:4])} not ready after {timeout} s, see {log}")


def stop(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=15)
    except Exception:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()


class Truth:
    """The simulator's state as PDDL facts, plus the item poses."""

    def __init__(self, node):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        from lab_replica import config, objects
        from lab_replica.execution.scene_state import ModelPoses
        from lab_replica.robot.kinematics import ARM_JOINTS, URKinematics
        self.rclpy = rclpy
        self.node = node
        self.cell = config.load_cell()
        self.items = objects.load_items()
        self.basket = objects.load_basket(self.cell)
        self.kin = URKinematics(self.cell["robot"]["ur_type"])
        self.poses = ModelPoses(node)
        self.st = {"grip": False, "q": None, "w": 0.1}
        self.joints = ARM_JOINTS
        node.create_subscription(Bool, "/rg6/grip_detected", lambda m: self.st.__setitem__("grip", m.data), 10)
        node.create_subscription(JointState, "/joint_states", self._js, 10)

    def _js(self, msg):
        from lab_replica.robot import rg6
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in self.joints):
            self.st["q"] = np.array([d[j] for j in self.joints])
        if "left_finger_slide" in d:
            self.st["w"] = rg6.width_from_slides(d["left_finger_slide"], d["right_finger_slide"])

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def read(self):
        """(facts, {item: (xyz, R)})."""
        from lab_replica.planning import pddl
        from lab_replica.robot import rg6
        from lab_replica.robot.collision import object_box
        from lab_replica.transforms import quat_to_matrix
        self.poses.poses.clear()
        self.spin(1.0)
        P = {}
        for it, m in self.items.items():
            if m.model in self.poses.poses:
                xyz, q = self.poses.poses[m.model]
                P[it] = (np.array(xyz), quat_to_matrix(q))
        bxy = self.poses.poses.get("basket", (np.array(self.cell["basket"]["center_xy"] + [0]), None))[0][:2]
        T = self.kin.fk(self.st["q"])
        tip = T[:3, 3] + T[:3, 2] * (rg6.depth_from_width(self.st["w"]) - 0.02)
        facts, held = set(), None
        if self.st["grip"]:
            best = None
            for it, (xyz, R) in P.items():
                b = object_box(self.items[it], xyz, R, it)
                L = (tip - b.c) @ b.R
                d = np.linalg.norm(np.maximum(np.abs(L) - b.h, 0))
                if best is None or d < best[0]:
                    best = (d, it)
            if best and best[0] < GRIP_DIST:
                held = best[1]
                facts.add(("robot-gripping", held))
        if held is None:
            facts.add(pddl.EMPTY)
        rim = self.cell["table"]["surface_z"] + self.basket.height
        for it, (xyz, R) in P.items():
            if it == held:
                continue
            r = math.hypot(xyz[0] - bxy[0], xyz[1] - bxy[1])
            if r < self.basket.r_top and xyz[2] < rim + 0.08:
                facts.add(("in-basket", it))
            elif xyz[2] < 0.25:
                facts.add(("on-table", it))
        return frozenset(facts), P

    @staticmethod
    def tilt_change(R, R_before):
        """Angle [deg] by which an item's up axis turned."""
        a, b = R[:, 2], R_before[:, 2]
        return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1, 1))))

    def upright(self, item, R):
        """Smallest tilt [deg] between the item and one of its resting poses."""
        from lab_replica.transforms import rpy_to_matrix
        m = self.items[item]
        best = 180.0
        for rest in m.spec["stable_poses"]:
            Rr = rpy_to_matrix(*m.poses[rest].rpy)
            z_rest = Rr.T @ np.array([0, 0, 1.0])     # world up, in the item frame, when rested
            best = min(best, math.degrees(math.acos(np.clip(np.dot(R @ z_rest, [0, 0, 1.0]), -1, 1))))
        return best


def choose(facts, counts, rng, refused=()):
    from lab_replica.planning import pddl
    held = pddl.held(facts)
    if held is None:
        on_table = [f[1] for f in facts if f[0] == "on-table"]
        if not on_table:
            return None
        on_table = [i for i in on_table if f"(pick-up {i})" not in refused] or on_table
        m = min(counts["pick-up"].get(i, 0) for i in on_table)
        return f"(pick-up {rng.choice(sorted(i for i in on_table if counts['pick-up'].get(i, 0) == m))})"
    n_basket = sum(1 for f in facts if f[0] == "in-basket")
    nb, nt = sum(counts["put-in-basket"].values()), sum(counts["put-on-table"].values())
    kind = "put-on-table" if n_basket >= 5 or nt < nb else "put-in-basket"
    if f"({kind} {held})" in refused:
        kind = "put-in-basket" if kind == "put-on-table" else "put-on-table"
    return f"({kind} {held})"


def expected(facts, action):
    from lab_replica.planning import pddl
    for a, s in pddl.successors(facts):
        if a == action:
            return s
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("problems", nargs="+")
    ap.add_argument("--actions", type=int, default=8, help="actions per problem")
    ap.add_argument("--domain", type=int, default=78, help="ROS_DOMAIN_ID (and Gazebo partition) to use")
    ap.add_argument("--report", default="verify_report.json")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--logs", default="/tmp/verify_logs")
    ap.add_argument("--sequence", nargs="+", default=None,
                    help="run these actions in order in every problem (instead of choosing; inapplicable ones are skipped)")
    args = ap.parse_args()
    os.makedirs(args.logs, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=f"verify{args.domain}")
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=f"verify{args.domain}")

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    from lab_replica import config
    from lab_replica.planning import pddl
    from lab_skills_msgs.action import ExecuteAction

    from rclpy.signals import SignalHandlerOptions
    # Ctrl-C / SIGTERM must end this program (stopping its simulation), not
    # only shut ROS down.
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("verify_actions")
    truth = Truth(node)
    client = ActionClient(node, ExecuteAction, "/lab_skills/execute")
    rng = random.Random(args.seed)
    counts = {"pick-up": {}, "put-in-basket": {}, "put-on-table": {}}
    rows = []
    if os.path.exists(args.report):
        rows = json.load(open(args.report))["actions"]
        for r in rows:
            k, it = r["action"].strip("()").split()
            counts[k][it] = counts[k].get(it, 0) + 1
    held_file = os.path.expanduser("~/.ros/lab_skills_held.json")
    for prob in args.problems:
        pdir = config.problem_dir(prob)
        name = os.path.basename(pdir.rstrip("/"))
        if any(r["problem"] == name for r in rows):
            print(f"{name}: already in the report, skipped")
            continue
        print(f"=== {name}")
        sim = server = None
        if os.path.exists(held_file):
            os.remove(held_file)
        try:
            sim = start(["ros2", "launch", "lab_replica", "sim.launch.py", f"problem:={pdir}", "gui:=false"], env,
                        f"{args.logs}/{name}_sim.log", "RG6 driver ready")
            server = start(["ros2", "run", "lab_replica", "skill_server.py", "--sim",
                            "--run-dir", os.path.join(config.workspace_root(), "runs", f"verify_{name}")], env,
                           f"{args.logs}/{name}_server.log", "ready: /lab_skills/execute")
            client.wait_for_server(timeout_sec=30)
            truth.poses.wait(15)
            facts, poses = truth.read()
            refused = set()   # actions refused in the current state
            todo = list(args.sequence) if args.sequence else None
            for k in range(len(todo) if todo else args.actions):
                if todo:
                    action = todo[k]
                    if expected(facts, action) is None:
                        print(f"  {k + 1}. {action}: not applicable in this state, skipped")
                        continue
                else:
                    action = choose(facts, counts, rng, refused)
                if action is None:
                    break
                kind, item = action.strip("()").split()
                exp = expected(facts, action)
                print(f"  {k + 1}. {action}", flush=True)
                t0 = time.time()
                fut = client.send_goal_async(ExecuteAction.Goal(action=action))
                while not fut.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                rf = fut.result().get_result_async()
                while not rf.done() and time.time() - t0 < 900:
                    rclpy.spin_once(node, timeout_sec=0.1)
                res = rf.result().result if rf.done() else None
                dt = time.time() - t0
                truth.spin(2.0)
                new, new_poses = truth.read()
                moved, disturbed = [], []
                for it, (xyz, R) in new_poses.items():
                    if it == item or it not in poses:
                        continue
                    d = float(np.linalg.norm(xyz - poses[it][0]))
                    tilt = truth.tilt_change(R, poses[it][1])
                    if d > MOVED or tilt > TILTED:
                        m = f"{it} moved {d * 100:.1f} cm, tilt {tilt:.0f} deg"
                        stays = ("in-basket", it) in facts and ("in-basket", it) in new
                        (disturbed if kind == "put-in-basket" and stays else moved).append(m)
                own_tilt = truth.upright(item, new_poses[item][1]) if item in new_poses else None
                row = dict(problem=name, action=action, server_ok=bool(res and res.success),
                           message=res.message if res else "timeout", seconds=round(dt, 1),
                           effect_ok=new == exp, unchanged_ok=not moved, others=moved, disturbed=disturbed,
                           missing=sorted(map(pddl.fact_str, exp - new)) if exp else [],
                           extra=sorted(map(pddl.fact_str, new - exp)) if exp else [],
                           item_tilt=None if own_tilt is None else round(own_tilt, 1))
                row["ok"] = row["server_ok"] and row["effect_ok"] and row["unchanged_ok"]
                rows.append(row)
                status = "OK" if row["ok"] else ("REFUSED" if not row["server_ok"] and new == facts else "FAIL")
                print(f"     {status} in {dt:.0f} s: {row['message']}"
                      + (f"; missing {row['missing']} extra {row['extra']}" if not row["effect_ok"] and row["server_ok"] else "")
                      + (f"; {'; '.join(moved)}" if moved else "")
                      + (f"; in the basket: {'; '.join(disturbed)}" if disturbed else ""), flush=True)
                if row["server_ok"]:
                    counts[kind][item] = counts[kind].get(item, 0) + 1
                refused = refused | {action} if new == facts else set()
                facts, poses = new, new_poses
                with open(args.report, "w") as f:
                    json.dump({"actions": rows}, f, indent=1)
        except KeyboardInterrupt:
            print("  interrupted")
            stop(server)
            stop(sim)
            raise
        except Exception as e:  # noqa: BLE001
            print(f"  ERROR: {e}")
        finally:
            stop(server)
            stop(sim)
            time.sleep(3)
    summarize(rows)
    rclpy.shutdown()


def summarize(rows):
    print("\n== summary")
    for kind in ("pick-up", "put-in-basket", "put-on-table"):
        rs = [r for r in rows if r["action"].startswith(f"({kind} ")]
        if not rs:
            continue
        ok = sum(r["ok"] for r in rs)
        refused = sum(not r["server_ok"] and not r["others"] for r in rs)
        items = sorted({r["action"].split()[1].rstrip(")") for r in rs if r["ok"]})
        print(f"{kind:14s} {ok}/{len(rs)} verified, {refused} refused by the robot, "
              f"{len(rs) - ok - refused} failed; mean {np.mean([r['seconds'] for r in rs]):.0f} s; "
              f"items done: {len(items)} ({', '.join(items)})")


if __name__ == "__main__":
    main()
