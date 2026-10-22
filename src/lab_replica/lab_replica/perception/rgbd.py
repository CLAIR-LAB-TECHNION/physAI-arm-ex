"""Frames from the wrist RealSense, and where its pixels are in the world.

The camera is read through the topics of the realsense2_camera driver (the
simulation bridges its sensor to the same names):
    colour          sensor_msgs/Image rgb8 (bgr8 is converted)
    aligned depth   sensor_msgs/Image, 16UC1 in mm (the driver) or 32FC1 in m
                    (Gazebo); depth aligned to the colour image
    camera_info     intrinsics of the colour image
On the real robot the camera runs at 640 x 360, 6 Hz (README): at full size
its images barely get across the lab network to this computer.
The camera's pose comes from the arm's joint angles and the hand-eye
transform in the robot description (cell.yaml wrist_camera), so frames are
only taken while the arm stands still. Whether a frame is new enough is
judged by when it ARRIVED here (this node's clock), not by its header stamp:
the camera's computer may run on another clock (in the lab the machines'
clocks differ by hours).
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ..robot.kinematics import Chain, robot_urdf
from ..transforms import inv_tf

OPTICAL_FRAME = "wrist_camera_color_optical_frame"
DEFAULT_TOPICS = {
    "color": "/camera/camera/color/image_raw",
    "depth": "/camera/camera/aligned_depth_to_color/image_raw",
    "info": "/camera/camera/color/camera_info",
}


@dataclass
class Frame:
    rgb: np.ndarray        # (H, W, 3) uint8
    depth: np.ndarray      # (H, W) float32 metres, NaN where unknown
    K: np.ndarray          # 3x3 intrinsics
    stamp: float           # s (when the colour image arrived, this node's clock)
    T_world_cam: np.ndarray = None   # optical frame in the world (set by the caller)
    q: np.ndarray = None             # arm joints when it was taken

    def points_world(self, mask=None, stride=1):
        """World points (N, 3) of the valid depth pixels (inside `mask`)."""
        H, W = self.depth.shape
        vv, uu = np.mgrid[0:H:stride, 0:W:stride]
        d = self.depth[::stride, ::stride]
        ok = np.isfinite(d) & (d > 0)
        if mask is not None:
            ok &= mask[::stride, ::stride]
        from ..exercise.state_estimation import pixels_to_world
        return pixels_to_world(uu[ok], vv[ok], d[ok], self.K, self.T_world_cam)

    def project(self, P):
        """World points -> (u, v) pixels and depth along the view."""
        T = inv_tf(self.T_world_cam)
        C = np.atleast_2d(P) @ T[:3, :3].T + T[:3, 3]
        u = self.K[0, 0] * C[:, 0] / C[:, 2] + self.K[0, 2]
        v = self.K[1, 1] * C[:, 1] / C[:, 2] + self.K[1, 2]
        return np.stack([u, v], axis=1), C[:, 2]


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def decode_image(msg):
    """sensor_msgs/Image -> numpy (RGB uint8, or depth float32 metres)."""
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8", "r8g8b8", "b8g8r8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[..., ::-1].copy() if enc in ("bgr8", "b8g8r8") else a.copy()
    if enc in ("16uc1", "mono16"):
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]
        d = a.astype(np.float32) / 1000.0
        d[a == 0] = np.nan
        return d
    if enc in ("32fc1", "r_float32"):
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, : msg.width].copy()
        a[~np.isfinite(a) | (a <= 0)] = np.nan
        return a
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


class RGBDCamera:
    """Keeps the newest colour and depth images of the wrist camera.

    `node` is spun by the caller (capture() spins it with spin_once)."""

    def __init__(self, node, ur_type, topics=None, max_skew=0.06):
        self.node = node
        t = dict(DEFAULT_TOPICS, **(topics or {}))
        self.color, self.depth, self.info = [], [], None
        self.max_skew = max_skew
        self.on_image = None    # f(colour msg, newest depth msg) for each colour image (the console's window)
        node.create_subscription(Image, t["color"], self._on_color, qos_profile_sensor_data)
        node.create_subscription(Image, t["depth"], lambda m: self._push(self.depth, m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, t["info"], self._on_info, qos_profile_sensor_data)
        self.chain = Chain(robot_urdf(ur_type), "world", OPTICAL_FRAME)

    def _push(self, buf, msg):
        """Keep the newest messages with their arrival time."""
        buf.append((msg, self.node.get_clock().now().nanoseconds * 1e-9))
        del buf[:-6]

    def _on_color(self, msg):
        self._push(self.color, msg)
        if self.on_image is not None and self.depth:
            self.on_image(msg, self.depth[-1][0])

    def _on_info(self, msg):
        self.info = msg

    def pose(self, q):
        """World pose of the colour optical frame for arm joints q."""
        return self.chain.fk(q)

    def capture(self, after, timeout=20.0):
        """The first colour/depth pair whose colour image arrived at or after
        `after` (this node's clock, s). Colour and depth are paired by their
        header stamps (both from the camera's clock)."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.info is None:
                continue
            for c, t_arrived in self.color:
                if t_arrived < after:
                    continue
                tc = _stamp(c)
                best = min(self.depth, key=lambda d: abs(_stamp(d[0]) - tc), default=None)
                if best is not None and abs(_stamp(best[0]) - tc) <= self.max_skew:
                    K = np.array(self.info.k, float).reshape(3, 3)
                    return Frame(decode_image(c), decode_image(best[0]), K, t_arrived)
        raise TimeoutError(f"no camera frame after t={after:.2f} within {timeout:.0f} s "
                           f"(colour {len(self.color)}, depth {len(self.depth)}, info {self.info is not None})")


def horizontal_fov(K, width):
    return 2 * math.atan(width / 2 / K[0, 0])
