"""Execute motion plans (see planning/skills.py) on the simulated or the real robot.

The arm is driven through a FollowJointTrajectory action:
  simulation:  /joint_trajectory_controller/follow_joint_trajectory
  real robot:  /passthrough_trajectory_controller/follow_joint_trajectory
               (ur_robot_driver; the robot interpolates each whole trajectory
               itself, which tolerates a jittery network such as Wi-Fi), or
               /scaled_joint_trajectory_controller/follow_joint_trajectory
The controller is the one the controller manager reports active
(active_trajectory_controller), so the same call works in simulation and on
the real robot. The gripper is driven through /rg6/gripper_command
(rg6_driver.py), in simulation and on the real robot alike.
Ctrl-C cancels the running goal: the arm stops (cancel_active()).
"""

import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..robot.kinematics import ARM_JOINTS


class ExecutionError(RuntimeError):
    pass


# Trajectory controllers, in order of preference when several are active.
TRAJECTORY_CONTROLLERS = ("passthrough_trajectory_controller", "scaled_joint_trajectory_controller",
                          "joint_trajectory_controller")


def active_trajectory_controller(node, timeout=60.0):
    """Name of the active arm trajectory controller (asks /controller_manager)."""
    cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            raise ExecutionError("no /controller_manager: is the simulation or the robot driver running?")
        t0 = time.time()
        while time.time() - t0 < timeout:
            # The controller manager drops a reply now and then when busy
            # ("failed to send response"): ask again after a few seconds.
            fut = cli.call_async(ListControllers.Request())
            t_call = time.time()
            while rclpy.ok() and not fut.done() and time.time() - t_call < 5.0:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not fut.done() or fut.result() is None:
                cli.remove_pending_request(fut)
                continue
            active = {c.name for c in fut.result().controller if c.state == "active"}
            for name in TRAJECTORY_CONTROLLERS:
                if name in active:
                    return name
            time.sleep(0.5)   # controllers may still be starting
        raise ExecutionError(f"none of {', '.join(TRAJECTORY_CONTROLLERS)} is active")
    finally:
        node.destroy_client(cli)


def _dur(t):
    sec = int(math.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class Executor(Node):
    def __init__(self, controller="joint_trajectory_controller", gripper_action="/rg6/gripper_command",
                 speed_scale=1.0):
        super().__init__("lab_executor")
        self.speed_scale = speed_scale
        self.q = None
        self.active = []   # goal handles that may still be running
        self.lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.traj = ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        self.grip = ActionClient(self, GripperCommand, gripper_action)

    def _on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            with self.lock:
                self.q = np.array([d[j] for j in ARM_JOINTS])

    def _spin_until(self, fut, timeout):
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                raise ExecutionError("timed out waiting for the action server")
        return fut.result()

    def wait_ready(self, timeout=60.0):
        t0 = time.time()
        while rclpy.ok() and self.q is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                raise ExecutionError("no /joint_states")
        if not self.traj.wait_for_server(timeout_sec=timeout):
            raise ExecutionError("trajectory action server not available")
        if not self.grip.wait_for_server(timeout_sec=timeout):
            raise ExecutionError("gripper action server not available")

    def current(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        with self.lock:
            return None if self.q is None else self.q.copy()

    # ---- arm ---------------------------------------------------------------

    def run_traj(self, points, times, label=""):
        pts = [np.array(p, float) for p in points]
        ts = list(times)
        q_now = self.current()
        if np.abs(pts[0] - q_now).max() > 0.02:
            # Start from where the robot is (small corrections only).
            if np.abs(pts[0] - q_now).max() > 0.25:
                raise ExecutionError(f"{label}: robot is {np.abs(pts[0] - q_now).max():.2f} rad away from the path start")
            pts.insert(0, q_now)
            ts = [0.0] + [t + 0.5 for t in ts]
        # The first point is the current state; JTC wants t > 0 for points.
        pts, ts = pts[1:], ts[1:]
        if not pts:
            return
        ts = [t / self.speed_scale for t in ts]
        traj = JointTrajectory(joint_names=list(ARM_JOINTS))
        vel = _velocities(pts, ts, q_now)
        for p, v, t in zip(pts, vel, ts):
            traj.points.append(JointTrajectoryPoint(positions=list(map(float, p)), velocities=list(map(float, v)),
                                                    time_from_start=_dur(t)))
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        gh = self._spin_until(self.traj.send_goal_async(goal), 10.0)
        if not gh.accepted:
            raise ExecutionError(f"{label}: trajectory rejected")
        self.active.append(gh)
        # Wall-clock timeout: also allow a simulation slowed down to 1/4 of real time (videos).
        res = self._spin_until(gh.get_result_async(), 4.0 * ts[-1] + 30.0)
        self.active.remove(gh)
        if res.result.error_code != 0:
            raise ExecutionError(f"{label}: trajectory failed ({res.result.error_code}: {res.result.error_string})")

    # ---- gripper -------------------------------------------------------------

    def gripper(self, width, force, label=""):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(force)
        gh = self._spin_until(self.grip.send_goal_async(goal), 10.0)
        if not gh.accepted:
            raise ExecutionError(f"{label}: gripper goal rejected")
        self.active.append(gh)
        res = self._spin_until(gh.get_result_async(), 40.0)   # the real RG6 driver retries slow reads
        self.active.remove(gh)
        return res.result

    def cancel_active(self, timeout=2.0):
        """Cancel the goals still running (after Ctrl-C): the arm stops."""
        futs = [gh.cancel_goal_async() for gh in self.active]
        t0 = time.time()
        while futs and not all(f.done() for f in futs) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        n = len(self.active)
        self.active.clear()
        return n

    # ---- plans -----------------------------------------------------------------

    def run_steps(self, steps, log=print, confirm=None):
        """confirm(text), if given, is called before every step (e.g. to wait for Enter)."""
        for st in steps:
            if confirm:
                confirm(f"arm: {st['label']}" if st["kind"] == "traj" else
                        f"gripper: {st['label']} -> {st['width'] * 1000:.0f} mm")
            if st["kind"] == "traj":
                log(f"  arm: {st['label']} ({len(st['points'])} waypoints, {st['times'][-1] / self.speed_scale:.1f} s)")
                self.run_traj(st["points"], st["times"], st["label"])
            else:
                log(f"  gripper: {st['label']} -> {st['width'] * 1000:.0f} mm")
                r = self.gripper(st["width"], st["force"], st["label"])
                if st.get("expect_grip") and not r.stalled:
                    raise ExecutionError(f"{st['label']}: no object between the fingers (width {r.position * 1000:.0f} mm)")
                if st.get("expect_grip"):
                    log(f"    holding, width {r.position * 1000:.1f} mm")


def _velocities(pts, ts, q0):
    """Finite-difference velocities, zero at the end."""
    P = [np.asarray(q0)] + list(pts)
    T = [0.0] + list(ts)
    v = []
    for i in range(1, len(P)):
        if i == len(P) - 1:
            v.append(np.zeros(6))
        else:
            v.append((P[i + 1] - P[i - 1]) / max(T[i + 1] - T[i - 1], 1e-6))
    return v
