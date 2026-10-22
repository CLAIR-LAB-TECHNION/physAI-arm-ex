#!/usr/bin/env python3
"""Record a video of the robot solving a task (solve.py) in simulation.

    python3 tools/record_task.py <task> out.mp4 [--domain 79] [--rtf 0.3]

<task> is a task of problems_tamp/ (make one with
`solve.py "<goal>" --plan-only --name <task>`). Run it in the workspace's
.venv with the workspace sourced. Like record_action.py, it starts the task's
world with the lab camera at 1920 x 1080, 30 Hz (the simulation runs at --rtf
of real time so every frame is rendered) and the skill server, plans the task
with Fast Downward, runs the setup pick-up (if the initial state has an item
in the gripper) unrecorded, then records the whole plan. The video is on
simulation time (it plays at real speed) and shows:

    the lab camera,
    top left: the goal and the plan, each action marked done / running / to do,
    insets: the wrist RealSense live, and the robot's last annotated view,
    a caption: the current action and what the robot is doing,
    at the end: whether the goal holds by the simulator's ground truth.

The ground truth and every action's result are also written to <out>.json.
"""

import argparse
import json
import os
import shutil
import signal
import sys
import tempfile
import textwrap
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from record_action import FPS, H, W, Recorder, put_text, video_world  # noqa: E402

PANEL_W = 600
END_SECONDS = 4.0
COLORS = {"done": (120, 230, 120), "running": (80, 220, 255), "failed": (90, 90, 255), "todo": (190, 190, 190)}


class TaskRecorder(Recorder):
    def __init__(self, node, out, goal, plan):
        super().__init__(node, out, "")
        self.goal_lines = textwrap.wrap(goal, 46)
        self.plan = plan
        self.status = ["todo"] * len(plan)
        self.current = None

    def on_action(self, k, action, status):
        if k < 0:
            return
        self.status[k] = "done" if status == "ok" else status
        self.current = k

    def panel(self, frame):
        n_lines = len(self.goal_lines) + len(self.plan) + 4
        h = 24 + 34 * n_lines
        x0, y0 = 16, 16
        roi = frame[y0:y0 + h, x0:x0 + PANEL_W]
        frame[y0:y0 + h, x0:x0 + PANEL_W] = (roi * 0.35).astype(np.uint8)
        y = y0 + 38
        put_text(frame, "GOAL", (x0 + 16, y), 0.75, (255, 255, 255), 2)
        for line in self.goal_lines:
            y += 34
            put_text(frame, line, (x0 + 16, y), 0.7, (255, 230, 160), 2)
        y += 50
        put_text(frame, f"PLAN (Fast Downward, {len(self.plan)} actions)", (x0 + 16, y), 0.75, (255, 255, 255), 2)
        for k, (a, st) in enumerate(zip(self.plan, self.status)):
            y += 34
            mark = {"done": "ok", "running": ">>", "failed": "X", "todo": "  "}[st]
            put_text(frame, f"{mark:>2} {k + 1}. {a}", (x0 + 16, y), 0.7, COLORS[st], 2)

    def compose(self, lab):
        frame = super().compose(lab)
        cv2.rectangle(frame, (0, H - 92), (W, H), (25, 25, 25), -1)
        if self.current is not None:
            put_text(frame, f"{self.current + 1}/{len(self.plan)}  {self.plan[self.current]}", (24, H - 52),
                     1.15, (120, 220, 255), 2)
        put_text(frame, self.phase, (24, H - 16), 0.85, (235, 235, 235), 2)
        self.panel(frame)
        return frame

    def end_card(self, text, ok, seconds=END_SECONDS):
        """Hold the last frame with the result on it."""
        with self.lock:
            if self.writer is None or self.last is None:
                return
            img = self.last.copy()
            cv2.rectangle(img, (0, H // 2 - 70), (W, H // 2 + 70), (25, 25, 25), -1)
            size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 3)[0]
            put_text(img, text, ((W - size[0]) // 2, H // 2 + 18), 1.6, (120, 230, 120) if ok else (90, 90, 255), 3)
            for _ in range(int(seconds * FPS)):
                self.writer.write(img)
                self.n += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--domain", type=int, default=79)
    ap.add_argument("--rtf", type=float, default=0.3)
    ap.add_argument("--logs", default="/tmp/record_logs")
    args = ap.parse_args()
    os.makedirs(args.logs, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=f"record{args.domain}")
    env = dict(os.environ)

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    from lab_replica import config
    from lab_replica.planning import pddl
    from lab_replica.tamp import task
    from lab_replica.tamp.execute import SkillClient, execute_plan
    from lab_replica.tamp.goal import holds, to_text, unsatisfied_facts
    from lab_replica.tamp.sim_truth import SimTruth
    from sim_run import start, stop

    tdir = task.task_dir(args.task)
    name = os.path.basename(tdir.rstrip("/"))
    init, formula, _ = task.read_task(tdir)
    plan = task.solve(init, formula, log=print)
    print(f"{name}: {to_text(formula)}\nplan: {' '.join(plan)}", flush=True)
    work = tempfile.mkdtemp(prefix="record_task_")
    world = video_world(tdir, work, args.rtf)
    held_file = os.path.expanduser("~/.ros/lab_skills_held.json")
    if os.path.exists(held_file):
        os.remove(held_file)
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("record_task")
    truth = SimTruth(node)
    skills = SkillClient(node, log=lambda s: print(s, flush=True))
    rec = TaskRecorder(node, os.path.abspath(args.out), to_text(formula), plan)
    sim = server = None
    result = {"task": name, "goal": to_text(formula), "init": sorted(map(pddl.fact_str, init)), "plan": plan}

    def on_action(k, action, status):
        rec.on_action(k, action, status)
        if k == 0 and status == "running" and not rec.recording:
            # The video starts after the setup: show the initial scene for a
            # moment before the first action is sent.
            rec.start()
            truth.spin(1.0 / args.rtf)

    try:
        sim = start(["ros2", "launch", "lab_replica", "sim.launch.py", f"world:={world}", "gui:=false",
                     "wrist_camera_rate:=10"], env, f"{args.logs}/{name}_sim.log", "RG6 driver ready")
        server = start(["ros2", "run", "lab_replica", "skill_server.py", "--sim",
                        "--run-dir", os.path.join(config.workspace_root(), "runs", f"video_{name}")], env,
                       f"{args.logs}/{name}_server.log", "ready: /lab_skills/execute")
        skills.wait(30.0)
        truth.available(15.0)
        t0 = time.time()
        report = execute_plan(skills, init, plan, log=lambda s: print(s, flush=True), on_action=on_action)
        truth.spin(1.5 / args.rtf)
        rec.recording = False
        facts = truth.facts()
        ok = holds(formula, facts)
        result.update(report, seconds_wall=round(time.time() - t0, 1), goal_holds=ok,
                      true_state=sorted(map(pddl.fact_str, facts)))
        if ok and report["ok"]:
            card = "Goal reached (simulator ground truth)"
        elif report["ok"]:
            card = "Plan executed, goal NOT reached: missing " + " ".join(unsatisfied_facts(formula, facts))
        else:
            card = f"Stopped: {report['actions'][-1]['action']} failed"
        rec.end_card(card, ok and report["ok"])
        n = rec.close()
        result.update(frames=n, video_seconds=round(n / FPS, 1))
        print(json.dumps({k: v for k, v in result.items() if k != "true_state"}, indent=1), flush=True)
    finally:
        rec.recording = False
        rec.close()
        stop(server)
        stop(sim)
        shutil.rmtree(work, ignore_errors=True)
        with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
            json.dump(result, f, indent=1)
        rclpy.shutdown()
    return 0 if result.get("goal_holds") and result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
