"""Forward and inverse kinematics for UR arms.

Forward kinematics walks the joint chain of the URDF built from
description/urdf/lab_robot.urdf.xacro, so it matches the simulation and the
robot_state_publisher exactly. Inverse kinematics is the closed-form UR
solution (up to 8 branches) in the DH frames, mapped to the URDF frames with
fixed offsets that are computed once against the URDF chain, then polished
with a few Gauss-Newton steps on the URDF chain.
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from ..transforms import inv_tf, make_tf, rpy_to_matrix, wrap

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
JOINT_WEIGHTS = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.3])


def axis_rotation(axis, q):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def robot_urdf(ur_type):
    """Expanded URDF text of the lab robot (no ros2_control blocks)."""
    from .. import config
    xf = config.path("description", "urdf", "lab_robot.urdf.xacro")
    try:
        import xacro
        return xacro.process_file(xf, mappings={"ur_type": ur_type}).toxml()
    except ImportError:
        return subprocess.run(["xacro", xf, f"ur_type:={ur_type}"], check=True,
                              capture_output=True, text=True).stdout


class Chain:
    def __init__(self, urdf_text, base="world", tip="rg6_base_link"):
        root = ET.fromstring(urdf_text)
        joints = {}
        for j in root.findall("joint"):
            child = j.find("child").get("link")
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0]
            joints[child] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                                 T=make_tf(rpy_to_matrix(*rpy), xyz), axis=axis)
        chain = []
        link = tip
        while link != base:
            j = joints[link]
            chain.append(j)
            link = j["parent"]
        self.chain = chain[::-1]
        self.limits = {}
        for j in root.findall("joint"):
            lim = j.find("limit")
            if lim is not None and j.get("name") in ARM_JOINTS:
                self.limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))

    def fk(self, q):
        """q: dict or sequence in ARM_JOINTS order."""
        if not isinstance(q, dict):
            q = dict(zip(ARM_JOINTS, q))
        T = np.eye(4)
        for j in self.chain:
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ make_tf(axis_rotation(j["axis"], q.get(j["name"], 0.0)))
        return T


class URKinematics:
    """IK/FK for the frame `tip` of the lab robot, in the world frame."""

    def __init__(self, ur_type, tip="rg6_base_link", urdf_text=None):
        urdf_text = urdf_text or robot_urdf(ur_type)
        self.chain = Chain(urdf_text, "world", tip)
        share = _ur_description_share()
        with open(os.path.join(share, "config", ur_type, "default_kinematics.yaml")) as f:
            k = yaml.safe_load(f)["kinematics"]
        self.d1 = k["shoulder"]["z"]
        self.a2 = k["forearm"]["x"]
        self.a3 = k["wrist_1"]["x"]
        self.d4 = k["wrist_1"]["z"]
        self.d5 = -k["wrist_2"]["y"]
        self.d6 = k["wrist_3"]["y"]
        # Fixed offsets: world -> DH base and DH end -> tip.
        q0 = np.zeros(6)
        self._calibrate(q0)

    # ---- DH model ------------------------------------------------------

    def _dh(self, a, alpha, d, th):
        ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(th), math.sin(th)
        return np.array([[ct, -st * ca, st * sa, a * ct],
                         [st, ct * ca, -ct * sa, a * st],
                         [0, sa, ca, d],
                         [0, 0, 0, 1]])

    def _dh_params(self):
        return ([0, self.a2, self.a3, 0, 0, 0],
                [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0],
                [self.d1, 0, 0, self.d4, self.d5, self.d6])

    def fk_dh(self, q):
        a, al, d = self._dh_params()
        T = np.eye(4)
        for i in range(6):
            T = T @ self._dh(a[i], al[i], d[i], q[i])
        return T

    def _calibrate(self, q0):
        """Find B and E with world_T_tip(q) = B @ fk_dh(q) @ E for all q.

        B is the DH base frame: the parent frame of the shoulder joint with
        the joint's rotation applied (its translation is d1, which the DH
        model already contains). E follows from one configuration.
        """
        Tb = np.eye(4)
        for j in self.chain.chain:
            if j["name"] == "shoulder_pan_joint":
                sh = j
                break
            Tb = Tb @ j["T"]
        self.B = Tb @ make_tf(sh["T"][:3, :3], (0, 0, 0))
        self.E = inv_tf(self.B @ self.fk_dh(q0)) @ self.chain.fk(q0)
        # Verify on random configurations.
        rng = np.random.default_rng(0)
        for _ in range(5):
            q = rng.uniform(-3, 3, 6)
            err = np.abs(self.B @ self.fk_dh(q) @ self.E - self.chain.fk(q)).max()
            if err > 1e-6:
                raise RuntimeError(f"DH model does not match URDF (err {err:.2e})")

    # ---- public --------------------------------------------------------

    def fk(self, q):
        return self.chain.fk(q)

    def ik_all(self, T_world_tip):
        """All closed-form solutions (list of 6-arrays, angles in [-pi, pi])."""
        T = inv_tf(self.B) @ np.asarray(T_world_tip) @ inv_tf(self.E)
        a2, a3, d4, d6 = self.a2, self.a3, self.d4, self.d6
        sols = []
        p05 = T @ np.array([0, 0, -d6, 1])
        R = math.hypot(p05[0], p05[1])
        if R < 1e-9 or abs(d4) > R:
            return sols
        phi = math.atan2(p05[1], p05[0])
        psi = math.acos(d4 / R)
        for s1 in (1, -1):
            t1 = phi + s1 * psi + math.pi / 2
            c1, s1v = math.cos(t1), math.sin(t1)
            v = (T[0, 3] * s1v - T[1, 3] * c1 - d4) / d6
            if abs(v) > 1 + 1e-9:
                continue
            v = max(-1.0, min(1.0, v))
            for s5 in (1, -1):
                t5 = s5 * math.acos(v)
                st5 = math.sin(t5)
                if abs(st5) < 1e-9:
                    t6 = 0.0
                else:
                    t6 = math.atan2((-T[0, 1] * s1v + T[1, 1] * c1) / st5,
                                    (T[0, 0] * s1v - T[1, 0] * c1) / st5)
                a, al, d = self._dh_params()
                T01 = self._dh(a[0], al[0], d[0], t1)
                T45 = self._dh(a[4], al[4], d[4], t5)
                T56 = self._dh(a[5], al[5], d[5], t6)
                T14 = inv_tf(T01) @ T @ inv_tf(T45 @ T56)
                p13 = T14 @ np.array([0, -d4, 0, 1]) - np.array([0, 0, 0, 1])
                n = np.linalg.norm(p13[:3])
                c3 = (n * n - a2 * a2 - a3 * a3) / (2 * a2 * a3)
                if abs(c3) > 1 + 1e-9:
                    continue
                c3 = max(-1.0, min(1.0, c3))
                for s3 in (1, -1):
                    t3 = s3 * math.acos(c3)
                    t2 = -math.atan2(p13[1], -p13[0]) + math.asin(max(-1, min(1, a3 * math.sin(t3) / n)))
                    T12 = self._dh(a[1], al[1], d[1], t2)
                    T23 = self._dh(a[2], al[2], d[2], t3)
                    T34 = inv_tf(T12 @ T23) @ T14
                    t4 = math.atan2(T34[1, 0], T34[0, 0])
                    q = np.array([wrap(x) for x in (t1, t2, t3, t4, t5, t6)])
                    q = self.refine(q, T_world_tip)
                    if q is not None:
                        sols.append(q)
        return sols

    def refine(self, q, T_target, iters=8, tol=1e-6):
        q = np.array(q, float)
        for _ in range(iters):
            e = self._pose_error(self.fk(q), T_target)
            if np.linalg.norm(e) < tol:
                return q
            J = self._jacobian(q, T_target)
            q = q - np.linalg.lstsq(J, e, rcond=None)[0]
        e = self._pose_error(self.fk(q), T_target)
        return q if np.linalg.norm(e) < 1e-4 else None

    def _pose_error(self, T, Tt):
        dp = T[:3, 3] - Tt[:3, 3]
        dR = Tt[:3, :3].T @ T[:3, :3]
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2
        return np.concatenate([dp, w])

    def _jacobian(self, q, Tt, h=1e-6):
        e0 = self._pose_error(self.fk(q), Tt)
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, float)
            dq[i] += h
            J[:, i] = (self._pose_error(self.fk(dq), Tt) - e0) / h
        return J

    def ik_near(self, T_world_tip, q_ref, max_jump=None):
        """Solution closest to q_ref (joint-space distance, allowing 2*pi
        shifts within the joint limits). None if unreachable."""
        best, best_d = None, float("inf")
        q_ref = np.asarray(q_ref, float)
        for q in self.ik_all(T_world_tip):
            q = self._unwrap_near(q, q_ref)
            if q is None:
                continue
            # Turning the last wrist joint is harmless; a branch change of the
            # other joints is not, so wrist_3 counts less.
            d = (np.abs(q - q_ref) * JOINT_WEIGHTS).max()
            if d < best_d:
                best, best_d = q, d
        if best is None or (max_jump is not None and best_d > max_jump):
            return None
        return best

    def _unwrap_near(self, q, q_ref):
        out = np.array(q, float)
        for i, name in enumerate(ARM_JOINTS):
            lo, hi = self.chain.limits.get(name, (-2 * math.pi, 2 * math.pi))
            cands = [out[i] + k * 2 * math.pi for k in (-2, -1, 0, 1, 2)]
            cands = [c for c in cands if lo - 1e-9 <= c <= hi + 1e-9]
            if not cands:
                return None
            out[i] = min(cands, key=lambda c: abs(c - q_ref[i]))
        return out


def _ur_description_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("ur_description")
    except Exception:
        return "/opt/ros/humble/share/ur_description"
