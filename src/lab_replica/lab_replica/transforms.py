"""Rotation and homogeneous-transform helpers (numpy only).

RPY follows the URDF/SDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
Quaternions are (x, y, z, w) as in ROS messages.
"""

import math

import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_matrix(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R):
    R = np.asarray(R)
    sy = -R[2, 0]
    if abs(sy) >= 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2, sy)
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        pitch = math.asin(sy)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    R = np.asarray(R)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def make_tf(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def tf_from_xyz_rpy(xyz, rpy):
    return make_tf(rpy_to_matrix(*rpy), xyz)


def inv_tf(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def angle_between(R1, R2):
    """Rotation angle (rad) of R1^T R2."""
    c = (np.trace(np.asarray(R1).T @ np.asarray(R2)) - 1) / 2
    return math.acos(max(-1.0, min(1.0, c)))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
