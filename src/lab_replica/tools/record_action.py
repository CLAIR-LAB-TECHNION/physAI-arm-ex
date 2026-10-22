#!/usr/bin/env python3
"""Record a video of the robot executing one PDDL action autonomously, in simulation.

    python3 tools/record_action.py <problem> "(put-on-table roll-of-blue-tape)" out.mp4
        [--setup "(pick-up roll-of-blue-tape)"] [--domain 79] [--rtf 0.3]

Run it in the workspace's .venv with the workspace sourced. It starts the
problem's world with the lab camera at 1920 x 1080, 30 Hz (the simulation
runs at --rtf of real time so every frame is rendered), the wrist camera at
10 Hz and the skill server; runs the --setup actions unrecorded (e.g. the
pick-up before a put); then records the action. The video is on simulation
time (it plays at real speed) and shows:

    the lab camera (the view of the lab photos),
    insets: the wrist RealSense live, and the robot's last annotated view
            of this action (pick-up: what SAM 3 found and where it estimates
            each item; put: where it will put the item, and why there),
    a caption: the action and what the robot is doing.

At the end the simulator's ground truth is checked as in sim_run.py (Truth)
and printed (and written next to the video as <out>.json).
"""

import argparse
import json
import os
import re
import shutil
import signal
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

W, H, FPS = 1920, 1080, 30
INSET_W = 640


def video_world(problem_dir, out_dir, rtf):
    """The problem's world with the lab camera at 1080p/30 Hz and a slower clock."""
    s = open(os.path.join(problem_dir, "world.sdf")).read()
    s = s.replace("<real_time_factor>1.0</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>", 1)
    i = s.index('<sensor name="rgb" type="camera">')
    j = s.index("</sensor>", i)
    cam = s[i:j]
    cam = re.sub(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{FPS}</update_rate>", cam)
    cam = cam.replace("<width>1280</width><height>720</height>", f"<width>{W}</width><height>{H}</height>")
    cam = cam.replace("<format>R8G8B8</format></image>", "<format>R8G8B8</format></image><anti_aliasing>4</anti_aliasing>")
    s = s[:i] + cam + s[j:]
    path = os.path.join(out_dir, "world.sdf")
    with open(path, "w") as f:
        f.write(s)
    return path


def put_text(img, text, org, scale, color=(255, 255, 255), thick=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class Recorder:
    """Composes and writes frames on the simulation-time timeline of the lab camera."""

    def __init__(self, node, out, action):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        from lab_replica.perception.rgbd import decode_image
        self.decode = decode_image
        self.out, self.action = out, action
        self.writer = None
        self.recording = False
        self.t0 = None
        self.n = 0
        self.last = None
        self.wrist = None
        self.det = None
        self.phase = ""
        self.lock = threading.Lock()
        node.create_subscription(Image, "/lab_camera/image", self._lab, QoSProfile(depth=60, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(Image, "/camera/camera/color/image_raw", self._wrist, qos_profile_sensor_data)
        node.create_subscription(Image, "/lab_skills/detections", self._det, 2)
        node.create_subscription(String, "/lab_skills/status", self._status, 10)

    def _wrist(self, m):
        self.wrist = m

    def _det(self, m):
        self.det = m

    def _status(self, m):
        self.phase = m.data

    def start(self):
        """Record from now on; views annotated before (e.g. by the setup
        actions) are not shown."""
        self.det = None
        self.recording = True

    def _inset(self, msg, title):
        if msg is None:
            return None
        img = self.decode(msg)[:, :, ::-1]   # decode gives RGB; the video is BGR
        h = int(img.shape[0] * INSET_W / img.shape[1])
        img = cv2.resize(np.ascontiguousarray(img), (INSET_W, h), interpolation=cv2.INTER_AREA)
        if title:
            put_text(img, title, (10, 26), 0.65)
        return img

    def compose(self, lab):
        frame = lab[:, :, ::-1].copy() if lab.shape[2] == 3 else lab
        y = 16
        for msg, title in ((self.wrist, "wrist RealSense (live)"), (self.det, None)):
            ins = self._inset(msg, title)
            if ins is None:
                continue
            h = ins.shape[0]
            x = W - INSET_W - 16
            frame[y - 3:y + h + 3, x - 3:x + INSET_W + 3] = 255
            frame[y:y + h, x:x + INSET_W] = ins
            y += h + 16
        cv2.rectangle(frame, (0, H - 92), (W, H), (25, 25, 25), -1)
        put_text(frame, self.action, (24, H - 52), 1.15, (120, 220, 255), 2)
        put_text(frame, self.phase, (24, H - 16), 0.85, (235, 235, 235), 2)
        return frame

    def _lab(self, m):
        if not self.recording:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = self.compose(self.decode(m))
        with self.lock:
            if self.writer is None:
                self.writer = cv2.VideoWriter(self.out, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
                self.t0 = t
            slot = int(round((t - self.t0) * FPS))
            if slot < self.n:
                return
            while self.n < slot and self.last is not None:
                self.writer.write(self.last)
                self.n += 1
            self.writer.write(img)
            self.n += 1
            self.last = img

    def close(self):
        with self.lock:
            if self.writer is not None:
                self.writer.release()
        return self.n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("problem")
    ap.add_argument("action")
    ap.add_argument("out")
    ap.add_argument("--setup", action="append", default=[], help="action(s) run first, not recorded")
    ap.add_argument("--domain", type=int, default=79)
    ap.add_argument("--rtf", type=float, default=0.3)
    ap.add_argument("--logs", default="/tmp/record_logs")
    args = ap.parse_args()
    os.makedirs(args.logs, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=f"record{args.domain}")
    env = dict(os.environ)

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    from lab_replica import config
    from lab_replica.planning import pddl
    from lab_skills_msgs.action import ExecuteAction
    from sim_run import Truth, expected, start, stop

    pdir = config.problem_dir(args.problem)
    work = tempfile.mkdtemp(prefix="record_")
    world = video_world(pdir, work, args.rtf)
    held_file = os.path.expanduser("~/.ros/lab_skills_held.json")
    if os.path.exists(held_file):
        os.remove(held_file)
    from rclpy.signals import SignalHandlerOptions
    # Ctrl-C / SIGTERM must end this program (stopping its simulation), not
    # only shut ROS down.
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("record_action")
    truth = Truth(node)
    rec = Recorder(node, os.path.abspath(args.out), args.action)
    client = ActionClient(node, ExecuteAction, "/lab_skills/execute")
    sim = server = None
    name = os.path.basename(pdir.rstrip("/"))
    result = {"problem": name, "action": args.action, "setup": args.setup}

    def run(action):
        fut = client.send_goal_async(ExecuteAction.Goal(action=action))
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        rf = fut.result().get_result_async()
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(node, timeout_sec=0.02)
        return rf.result().result

    try:
        sim = start(["ros2", "launch", "lab_replica", "sim.launch.py", f"world:={world}", "gui:=false",
                     "wrist_camera_rate:=10"], env, f"{args.logs}/{name}_sim.log", "RG6 driver ready")
        server = start(["ros2", "run", "lab_replica", "skill_server.py", "--sim",
                        "--run-dir", os.path.join(config.workspace_root(), "runs", f"video_{name}")], env,
                       f"{args.logs}/{name}_server.log", "ready: /lab_skills/execute")
        client.wait_for_server(timeout_sec=30)
        truth.poses.wait(15)
        for a in args.setup:
            r = run(a)
            print(f"setup {a}: {'OK' if r.success else 'FAILED'} {r.message}", flush=True)
            if not r.success:
                raise RuntimeError(f"setup failed: {r.message}")
        facts, poses = truth.read()
        exp = expected(facts, args.action)
        rec.phase = "starting"
        rec.start()
        truth.spin(1.0)
        t0 = time.time()
        r = run(args.action)
        dt = time.time() - t0
        truth.spin(1.5 / args.rtf)
        rec.recording = False
        n = rec.close()
        new, new_poses = truth.read()
        moved, disturbed = [], []
        kind, item = args.action.strip("()").split()
        for it, (xyz, R) in new_poses.items():
            if it != item and it in poses:
                d = float(np.linalg.norm(xyz - poses[it][0]))
                tilt = truth.tilt_change(R, poses[it][1])
                if d > 0.02 or tilt > 10:
                    m = f"{it} moved {d * 100:.1f} cm, tilt {tilt:.0f} deg"
                    stays = ("in-basket", it) in facts and ("in-basket", it) in new
                    (disturbed if kind == "put-in-basket" and stays else moved).append(m)
        result.update(server_ok=bool(r.success), message=r.message, seconds_wall=round(dt, 1),
                      frames=n, video_seconds=round(n / FPS, 1), effect_ok=new == exp, others=moved,
                      disturbed=disturbed,
                      missing=sorted(map(pddl.fact_str, exp - new)) if exp else [],
                      extra=sorted(map(pddl.fact_str, new - exp)) if exp else [])
        result["ok"] = result["server_ok"] and result["effect_ok"] and not moved
        print(json.dumps(result, indent=1), flush=True)
    finally:
        rec.recording = False
        rec.close()
        stop(server)
        stop(sim)
        shutil.rmtree(work, ignore_errors=True)
        with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
            json.dump(result, f, indent=1)
        rclpy.shutdown()
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
