"""Run a headless Ignition Gazebo (Fortress) server from Python, without ROS.

Used to settle item arrangements (problem generator, replanning) and to
render camera images. Every server gets a private IGN_PARTITION, so it never
interferes with a running simulation.
"""

import glob
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time

import numpy as np
from PIL import Image

from .. import config


def gazebo_env():
    env = dict(os.environ)
    paths = [config.path("models")]
    old = env.get("IGN_GAZEBO_RESOURCE_PATH", "")
    env["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(paths + ([old] if old else []))
    # Keep these runs off the network so they never talk to a running sim.
    env.setdefault("IGN_IP", "127.0.0.1")
    return env


def _parse_float_block(text, key):
    out = {}
    m = re.search(key + r"\s*\{([^}]*)\}", text)
    if not m:
        return None
    for k, v in re.findall(r"(\w+):\s*([-+0-9.eE]+)", m.group(1)):
        out[k] = float(v)
    return out


def parse_pose_v(text):
    """Parse `ign topic -e` output of a Pose_V message.

    Returns {name: (xyz ndarray, quat ndarray xyzw)} for the first pose of
    each name (top-level models come first and are in the world frame).
    """
    poses = {}
    for block in re.split(r"\npose \{|^pose \{", text):
        m = re.search(r'name:\s*"([^"]+)"', block)
        if not m:
            continue
        name = m.group(1)
        if name in poses:
            continue
        p = _parse_float_block(block, "position") or {}
        q = _parse_float_block(block, "orientation") or {}
        xyz = np.array([p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)])
        quat = np.array([q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 0.0)])
        if not np.any(quat):
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        poses[name] = (xyz, quat)
    return poses


class GzServer:
    """A headless `ign gazebo -s` process for one world file."""

    def __init__(self, world_file, world_name, rendering=False, verbose=1, partition=None):
        self.world_name = world_name
        self.env = gazebo_env()
        # A private partition keeps topics separate from any other simulation.
        self.env["IGN_PARTITION"] = partition or f"labgen_{os.getpid()}_{int(time.time() * 1000) % 100000}"
        cmd = ["ign", "gazebo", "-s", "-r", "-v", str(verbose)]
        if rendering:
            cmd.append("--headless-rendering")
        cmd.append(world_file)
        self.log = tempfile.NamedTemporaryFile(prefix="gz_", suffix=".log", delete=False)
        self.proc = subprocess.Popen(cmd, env=self.env, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def _topic_once(self, topic, timeout=10.0):
        try:
            out = subprocess.run(["ign", "topic", "-e", "-t", topic, "-n", "1"], env=self.env,
                                 capture_output=True, text=True, timeout=timeout)
            return out.stdout
        except subprocess.TimeoutExpired:
            return ""

    def sim_time(self):
        txt = self._topic_once(f"/world/{self.world_name}/stats", timeout=5.0)
        st = _parse_float_block(txt, "sim_time")
        if st is None:
            return None
        return st.get("sec", 0.0) + st.get("nsec", 0.0) * 1e-9

    def wait_until(self, t_sim, timeout=120.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.proc.poll() is not None:
                raise RuntimeError(f"gazebo exited early, see {self.log.name}")
            t = self.sim_time()
            if t is not None and t >= t_sim:
                return t
            time.sleep(0.2)
        raise TimeoutError(f"simulation did not reach t={t_sim} s (log: {self.log.name})")

    def poses(self):
        txt = self._topic_once(f"/world/{self.world_name}/pose/info", timeout=10.0)
        return parse_pose_v(txt)

    def grab_image(self, save_dir, out_png, topic="/lab_camera/image", settle=1.5):
        """The world's camera must save frames to save_dir. Subscribing makes
        the sensor render; the newest saved frame is copied to out_png."""
        before = set(glob.glob(os.path.join(save_dir, "*.png")))
        sub = subprocess.Popen(["ign", "topic", "-e", "-t", topic], env=self.env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t0 = time.time()
        try:
            while time.time() - t0 < 60:
                new = set(glob.glob(os.path.join(save_dir, "*.png"))) - before
                if len(new) >= 3:
                    time.sleep(settle)
                    break
                time.sleep(0.2)
        finally:
            sub.terminate()
            sub.wait()
        files = sorted(set(glob.glob(os.path.join(save_dir, "*.png"))) - before, key=os.path.getmtime)
        # The newest frame may still be being written: take the newest complete one.
        for f in reversed(files):
            try:
                with Image.open(f) as img:
                    img.load()
            except (OSError, SyntaxError):   # PIL raises SyntaxError for a broken chunk
                continue
            shutil.copy(f, out_png)
            return out_png
        raise RuntimeError(f"no complete camera frame rendered (log: {self.log.name})")

    def stop(self):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=10)
            except Exception:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
