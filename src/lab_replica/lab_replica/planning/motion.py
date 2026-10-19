"""Motion planning for top-down pick and place with the RG6.

Pure computation (no ROS): gripper poses, inverse kinematics along straight
lines and arcs of the gripper, and the timing of joint paths. Collision
checks are in skills.py.

Conventions:
  * The gripper always points straight down. Its yaw `psi` is the rotation of
    rg6_base_link about the world z axis; the fingers close along the
    gripper's y axis, i.e. along the world direction psi + pi/2.
  * `z_base` is the height of the RG6 mounting face (rg6_base_link origin).
    The fingertip ends are depth(width) below it.
"""

import math

import numpy as np

from ..robot.kinematics import URKinematics
from ..transforms import make_tf, rot_x, rot_z, wrap

R_DOWN = rot_x(math.pi)  # gripper z axis pointing down


def gripper_pose(x, y, z_base, psi):
    return make_tf(rot_z(psi) @ R_DOWN, (x, y, z_base))


def psi_for_closing(closing_yaw):
    """Gripper yaws whose finger motion is along closing_yaw (two options)."""
    p = wrap(closing_yaw - math.pi / 2)
    return [p, wrap(p + math.pi)]


def min_jerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class PlanError(RuntimeError):
    pass


class Planner:
    """Inverse kinematics along gripper paths, and their timing."""

    # Speeds used for timing, well below the UR limits (3.14 rad/s) so the
    # motions are safe on the real robot.
    V_JOINT = 0.8     # rad/s
    V_CART = 0.20     # m/s, transit
    V_SLOW = 0.05     # m/s, approach and retreat

    def __init__(self, cell):
        robot = cell["robot"]
        self.kin = URKinematics(robot["ur_type"])
        self.base_xy = np.array(robot["base_xyz"][:2], float)
        self.home = np.array(robot["home_joints"], float)

    # ---- inverse kinematics along paths ---------------------------------

    def ik(self, T, q_ref, max_jump=None):
        return self.kin.ik_near(T, q_ref, max_jump=max_jump)

    def line(self, q_start, p0, p1, psi0, psi1, step=0.01, max_jump=0.3):
        """Joint path for a straight line of the gripper from p0 to p1 (3D),
        yaw interpolated from psi0 to psi1."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        n = max(2, int(math.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
        dpsi = wrap(psi1 - psi0)
        path = [np.asarray(q_start, float)]
        for i in range(1, n):
            s = i / (n - 1)
            p = p0 + s * (p1 - p0)
            q = self.ik(gripper_pose(p[0], p[1], p[2], psi0 + s * dpsi), path[-1], max_jump=max_jump)
            if q is None:
                raise PlanError(f"line IK failed at {p.round(3)}")
            path.append(q)
        return path

    def arc(self, q_start, p0, p1, psi0, psi1, step=0.02, max_jump=0.3):
        """Path at constant height that moves around the robot base in polar
        coordinates (radius and bearing interpolated), so it never cuts
        close to the base axis."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        c = self.base_xy
        r0, r1 = np.linalg.norm(p0[:2] - c), np.linalg.norm(p1[:2] - c)
        a0 = math.atan2(p0[1] - c[1], p0[0] - c[0])
        da = wrap(math.atan2(p1[1] - c[1], p1[0] - c[0]) - a0)
        length = abs(da) * max(r0, r1) + abs(r1 - r0) + abs(p1[2] - p0[2])
        n = max(2, int(math.ceil(length / step)) + 1)
        dpsi = wrap(psi1 - psi0)
        path = [np.asarray(q_start, float)]
        for i in range(1, n):
            s = i / (n - 1)
            r = r0 + s * (r1 - r0)
            a = a0 + s * da
            z = p0[2] + s * (p1[2] - p0[2])
            x, y = c[0] + r * math.cos(a), c[1] + r * math.sin(a)
            q = self.ik(gripper_pose(x, y, z, psi0 + s * dpsi), path[-1], max_jump=max_jump)
            if q is None:
                raise PlanError(f"arc IK failed at ({x:.3f}, {y:.3f}, {z:.3f})")
            path.append(q)
        return path

    def joint_move(self, q0, q1, step=0.05):
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(2, int(math.ceil(np.abs(q1 - q0).max() / step)) + 1)
        return [q0 + (q1 - q0) * i / (n - 1) for i in range(n)]

    # ---- timing ------------------------------------------------------------

    def time_path(self, path, speed=None, t_min=0.6):
        """Times for a joint path: overall duration from the joint speed limit
        and the Cartesian speed, with a minimum-jerk profile along arc length."""
        Q = np.array(path)
        if len(Q) < 2:
            return [0.0]
        dq = np.abs(np.diff(Q, axis=0)).max(axis=1)
        P = np.array([self.kin.fk(q)[:3, 3] for q in Q])
        ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
        # Parameter: blend of joint and Cartesian distance.
        seg = np.maximum(dq / self.V_JOINT, ds / (speed or self.V_CART))
        total = max(t_min, seg.sum() * 1.875)  # 1.875 = peak/mean of min-jerk
        u = np.concatenate([[0], np.cumsum(seg)]) / max(seg.sum(), 1e-9)
        # Invert the min-jerk profile: t such that min_jerk(t/T) = u.
        ts = []
        grid = np.linspace(0, 1, 2001)
        mj = min_jerk(grid)
        for ui in u:
            ts.append(float(np.interp(ui, mj, grid)) * total)
        ts[0] = 0.0
        # Keep times strictly increasing.
        for i in range(1, len(ts)):
            ts[i] = max(ts[i], ts[i - 1] + 1e-3)
        return ts
