"""ROS 2 driver of the OnRobot RG6, the same interface for the simulated and
the real gripper (started by sim.launch.py / real.launch.py).

  action  /rg6/gripper_command   control_msgs/action/GripperCommand
          command.position   target width between the pads [m] (0 - 0.151)
          command.max_effort grip force [N] (25 - 120 on the real RG6; in
                             simulation the force is gripper.grip_force)
          result.stalled     true when the fingers stopped on an object
                             (the RG6 "grip detected" flag)
  topic   /rg6/width         std_msgs/Float64, measured width [m]
  topic   /rg6/grip_detected std_msgs/Bool

Parameters:
  backend     'sim' (Gazebo) or 'onrobot' (the real gripper)
  ip, port    'onrobot': the UR controller and the OnRobot URCap's XML-RPC port (41414)
  tool_index  'onrobot': OnRobot tool index (0 for a single gripper)
"""

import threading
import time
import xmlrpc.client

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray

from . import rg6

DEFAULT_FORCE = 40.0   # N, when a command gives none


class SimBackend:
    """Drives the simulated RG6 joints through /rg6_controller/commands.

    The finger slides move towards the target at SPEED and push with the force
    their effort limit allows (gripper.grip_force); the finger depth and the
    knuckles follow the measured opening along the real RG6 linkage curve.
    """

    SPEED = 0.08       # m/s per finger
    PERIOD = 0.005     # s, command rate
    # How far the common slide command may lead the mean finger position [m]:
    # more than the lag of freely closing fingers (about 2 mm), small compared
    # with the 12.5 mm squeeze.
    HOLD_AHEAD = 0.003

    def __init__(self, node):
        self.node = node
        self.q = {}              # latest gripper joint positions
        self.s_target = None     # target slide of both fingers [m]
        self.s_cmd = None        # ramped slide command [m]
        self.width = float("nan")
        self.grip = False
        self.cmd_pub = node.create_publisher(Float64MultiArray, "/rg6_controller/commands", 10)
        node.create_subscription(JointState, "/joint_states", self._on_joint_states, 50, callback_group=node.cb)
        node.create_timer(self.PERIOD, self._tick, callback_group=node.cb)

    def _on_joint_states(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in rg6.JOINT_ORDER:
                self.q[n] = p
        if "left_finger_slide" in self.q and "right_finger_slide" in self.q:
            self.width = rg6.width_from_slides(self.q["left_finger_slide"], self.q["right_finger_slide"])
            if self.s_cmd is None:
                self.s_cmd = self._mean_slide()

    def _mean_slide(self):
        return (self.q["left_finger_slide"] + self.q["right_finger_slide"]) / 2

    def _tick(self):
        if self.s_cmd is None:
            return
        if self.s_target is not None:
            step = self.SPEED * self.PERIOD
            self.s_cmd += max(-step, min(step, self.s_target - self.s_cmd))
        s_mean = self._mean_slide()
        # On the real RG6 both fingers hang on one linkage, so they stay
        # symmetric about the gripper axis. The two simulated slides are
        # independent: with both commands far beyond the object, fingers and
        # object would drift sideways together under any load. Once the
        # fingers are blocked, the common command is therefore held
        # HOLD_AHEAD past their mean, which still pushes with the full force
        # limit but pulls a drifted pair back.
        s_cmd = min(self.s_cmd, s_mean + self.HOLD_AHEAD)
        # Depth and knuckles follow the mean opening the way the real
        # parallelogram linkage does (both pads at the same depth).
        cmd = rg6.joint_command(rg6.q_from_slide(s_mean), s_left=s_cmd, s_right=s_cmd)
        self.cmd_pub.publish(Float64MultiArray(data=cmd))
        self.node.publish_state(self.width, self.grip)

    def execute(self, width, force, goal_handle):
        """Move to `width`. Returns (grip detected, cancelled)."""
        s_goal = rg6.slide_from_q(rg6.q_from_width(width))
        s_now = self._mean_slide()
        closing = s_goal > s_now
        self.grip = False
        # After a grip the old command lies beyond the blocked fingers;
        # restart the ramp from where the fingers really are.
        self.s_cmd = s_now
        self.s_target = s_goal
        t0 = time.time()
        still_since = None
        last_w = self.width
        while rclpy.ok():
            time.sleep(0.02)
            if goal_handle.is_cancel_requested:
                self.s_target = self.s_cmd = self._mean_slide()
                return False, True
            s_now = self._mean_slide()
            moving = abs(self.width - last_w) > 1e-4
            last_w = self.width
            ramp_done = abs(self.s_cmd - s_goal) < 1e-6
            if ramp_done and abs(s_now - s_goal) < 0.0008:
                return False, False
            if ramp_done and not moving:
                still_since = still_since or time.time()
                if time.time() - still_since > 0.3:
                    # Blocked before the target: an object is held when closing.
                    self.grip = closing and abs(s_now - s_goal) > 0.002
                    return self.grip, False
            else:
                still_since = None
            if time.time() - t0 > 10.0:
                # Still creeping after 10 s (e.g. a slow simulation): stopped
                # short of the target when closing means something is held.
                self.grip = closing and abs(s_now - s_goal) > 0.002
                return self.grip, False


class OnRobotBackend:
    """The real RG6 on the UR e-Series tool flange, through the OnRobot URCap.

    The URCap runs a daemon on the UR controller that serves XML-RPC on port
    41414, with the calls its URScript functions use: rg_grip(tool_index,
    width_mm, force_N), rg_stop(tool_index), rg_get_width / rg_get_busy /
    rg_get_grip_detected / rg_get_status(tool_index). Nothing has to be
    installed on the robot, and it works while ur_robot_driver controls the
    arm. Widths are in mm between the fingertips (with the fingertip offset
    set in the URCap). tools/fake_onrobot_xmlrpc.py imitates the daemon.

    On the lab robot the daemon now and then takes seconds to answer while
    the arm works (2 s timeouts were common). So: a 5 s timeout, no state
    polling while a command runs, the command sent again once if it times
    out (the same target, so that is harmless), and failed reads during a
    command are retried instead of failing it.
    """

    POLL = 0.1   # s
    COMMAND_TIMEOUT = 15.0   # s for a command to finish

    def __init__(self, node, ip, port, tool_index, timeout=5.0):
        self.node = node
        self.index = tool_index
        self.lock = threading.Lock()
        self.executing = False
        self.proxy = xmlrpc.client.ServerProxy(f"http://{ip}:{port}/", transport=_TimeoutTransport(timeout))
        log = node.get_logger()
        with self.lock:
            log.info(f"OnRobot URCap daemon {ip}:{port}: {self.proxy.get_discovery()}")
        w, busy, grip, self.status = self._state()
        log.info(f"RG6 width {w:.1f} mm, busy {busy}, grip detected {grip}, status {self.status}")
        self.width, self.grip = w / 1000.0, bool(grip)
        # The UR driver owns /joint_states; the launch file remaps this one
        # to /rg6/joint_states so RViz shows the real finger opening.
        self.js_pub = node.create_publisher(JointState, "/joint_states", 10)
        node.create_timer(self.POLL, self._poll, callback_group=node.cb)

    def _call(self, name, *args):
        with self.lock:
            return getattr(self.proxy, name)(self.index, *args)

    def _state(self):
        """(width_mm, busy, grip_detected, status) in one request."""
        with self.lock:
            mc = xmlrpc.client.MultiCall(self.proxy)
            for name in ("rg_get_width", "rg_get_busy", "rg_get_grip_detected", "rg_get_status"):
                getattr(mc, name)(self.index)
            return tuple(mc())

    def _poll(self):
        if self.executing:
            return
        try:
            w, busy, grip, status = self._state()
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().error(f"OnRobot XML-RPC read failed: {e}", throttle_duration_sec=5.0)
            return
        if status != self.status:
            self.node.get_logger().warn(f"RG6 status {self.status} -> {status}")
        self.width, self.grip, self.status = w / 1000.0, bool(grip), status
        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = list(rg6.JOINT_ORDER)
        js.position = rg6.joint_command(rg6.q_from_width(self.width))
        self.js_pub.publish(js)
        self.node.publish_state(self.width, self.grip)

    def execute(self, width, force, goal_handle):
        """Move to `width`. Returns (grip detected, cancelled)."""
        force = max(25.0, min(120.0, force if force > 0 else DEFAULT_FORCE))
        width_mm = max(0.0, min(160.0, width * 1000.0))
        log = self.node.get_logger()
        self.executing = True
        try:
            for attempt in (1, 2):
                try:
                    ret = self._call("rg_grip", float(width_mm), float(force))
                    break
                except Exception as e:  # noqa: BLE001
                    if attempt == 2:
                        raise
                    log.warn(f"rg_grip({width_mm:.1f} mm) failed ({e}); sending it again")
            log.info(f"rg_grip({width_mm:.1f} mm, {force:.0f} N) -> {ret}")
            # The busy flag rises once the fingers start; give it time, then wait
            # for it to drop (or the fingers to arrive, for a no-op command).
            t0 = time.time()
            read = False
            while time.time() - t0 < self.COMMAND_TIMEOUT:
                time.sleep(0.05)
                if goal_handle.is_cancel_requested:
                    self._call("rg_stop")
                    return False, True
                try:
                    w, busy, grip, _ = self._state()
                except Exception as e:  # noqa: BLE001
                    log.warn(f"OnRobot XML-RPC read failed during the command ({e}); retrying")
                    continue
                read = True
                self.width, self.grip = w / 1000.0, bool(grip)
                if not busy and (time.time() - t0 > 0.5 or abs(w - width_mm) < 1.0):
                    break
            else:
                log.error(f"RG6 still busy (or unreadable: {not read}) after {self.COMMAND_TIMEOUT:.0f} s")
            return self.grip, False
        finally:
            self.executing = False


class _TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = self.timeout
        return conn


class RG6Driver(Node):
    def __init__(self):
        super().__init__("rg6_driver")
        self.cb = ReentrantCallbackGroup()
        self.pub_width = self.create_publisher(Float64, "/rg6/width", 10)
        self.pub_grip = self.create_publisher(Bool, "/rg6/grip_detected", 10)
        backend = self.declare_parameter("backend", "sim").value
        if backend == "sim":
            self.backend = SimBackend(self)
        elif backend == "onrobot":
            ip = self.declare_parameter("ip", "").value
            port = self.declare_parameter("port", 41414).value
            index = self.declare_parameter("tool_index", 0).value
            if not ip:
                raise ValueError("backend 'onrobot' needs the robot address: -p ip:=<UR ip>")
            self.backend = OnRobotBackend(self, ip, port, index)
        else:
            raise ValueError(f"backend must be 'sim' or 'onrobot', not {backend!r}")
        self.server = ActionServer(
            self, GripperCommand, "/rg6/gripper_command", self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self.cb)
        self.get_logger().info(f"RG6 driver ready ({backend})")

    def publish_state(self, width, grip):
        self.pub_width.publish(Float64(data=width))
        self.pub_grip.publish(Bool(data=grip))

    def _execute(self, goal_handle):
        cmd = goal_handle.request.command
        self.get_logger().info(f"gripper command: width {cmd.position * 1000:.1f} mm, force {cmd.max_effort:.0f} N")
        stalled, cancelled = self.backend.execute(cmd.position, cmd.max_effort, goal_handle)
        width = self.backend.width
        res = GripperCommand.Result()
        res.position = float(width)
        res.effort = float(cmd.max_effort)
        res.stalled = bool(stalled)
        res.reached_goal = abs(width - cmd.position) < 0.003
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        self.get_logger().info(f"gripper done: width {width * 1000:.1f} mm, grip detected {stalled}")
        return res


def main():
    rclpy.init()
    node = RG6Driver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
