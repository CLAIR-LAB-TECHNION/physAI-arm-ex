"""Collision checks for the whole robot (arm links and gripper) against the
table, the basket and the objects.

The robot is represented by points sampled on the collision geometry of the
URDF (UR collision meshes, gripper boxes) and moved with forward kinematics.
Objects are oriented boxes around their collision shapes. This is
conservative and fast enough to check every waypoint of a plan.
"""

import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

from . import rg6
from .kinematics import ARM_JOINTS, axis_rotation
from ..transforms import make_tf, rpy_to_matrix

# Links that move with the fingers (pads and linkage); they may touch the
# object being grasped.
FINGER_LINKS = ("left_inner_finger", "right_inner_finger", "left_inner_knuckle", "right_inner_knuckle",
                "left_outer_knuckle", "right_outer_knuckle")


def _read_stl(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        vs = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
              if ln.strip().startswith("vertex")]
        return np.array(vs)
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float)


def resolve_uri(uri):
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(pkg), rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def _box_points(size, n=3):
    sx, sy, sz = (s / 2 for s in size)
    g = np.linspace(-1, 1, n)
    pts = []
    for a in g:
        for b in g:
            for sgn in (-1, 1):
                pts += [(sgn * sx, a * sy, b * sz), (a * sx, sgn * sy, b * sz), (a * sx, b * sy, sgn * sz)]
    return np.unique(np.round(np.array(pts), 6), axis=0)


def _cyl_points(r, h, n=12):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = []
    for z in (-h / 2, 0, h / 2):
        pts += [(r * math.cos(t), r * math.sin(t), z) for t in a]
    return np.array(pts)


class RobotPoints:
    """Sampled collision points of every link, in each link's frame."""

    def __init__(self, urdf_text, max_mesh_points=160):
        root = ET.fromstring(urdf_text)
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1.0, 0, 0])
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                T=make_tf(rpy_to_matrix(*rpy), xyz), axis=axis / np.linalg.norm(axis))
        self.points = {}
        rng = np.random.default_rng(0)
        for link in root.findall("link"):
            pts = []
            for c in link.findall("collision"):
                g = c.find("geometry")[0]
                o = c.find("origin")
                xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
                if g.tag == "mesh":
                    v = np.unique(np.round(_read_stl(resolve_uri(g.get("filename"))), 4), axis=0)
                    if len(v) > max_mesh_points:
                        v = v[rng.choice(len(v), max_mesh_points, replace=False)]
                elif g.tag == "box":
                    v = _box_points([float(s) for s in g.get("size").split()])
                elif g.tag == "cylinder":
                    v = _cyl_points(float(g.get("radius")), float(g.get("length")))
                else:
                    continue
                pts.append(v @ rpy_to_matrix(*rpy).T + np.array(xyz))
            if pts:
                self.points[link.get("name")] = np.vstack(pts)
        # Order links so parents come first.
        self.order = []
        seen = {"world"}
        pending = list(self.joints.keys())
        while pending:
            for ch in list(pending):
                if self.joints[ch]["parent"] in seen:
                    self.order.append(ch)
                    seen.add(ch)
                    pending.remove(ch)

    def frames(self, q_arm, width):
        q = dict(zip(ARM_JOINTS, q_arm))
        qg = rg6.q_from_width(width)
        s = rg6.slide_from_q(qg)
        lift = rg6.lift_from_q(qg)
        q.update({"left_finger_slide": s, "right_finger_slide": s, "left_finger_lift": lift,
                  "right_finger_lift": lift, "finger_joint": qg, "left_inner_knuckle_joint": -qg,
                  "right_outer_knuckle_joint": -qg, "right_inner_knuckle_joint": -qg})
        T = {"world": np.eye(4)}
        for ch in self.order:
            j = self.joints[ch]
            M = T[j["parent"]] @ j["T"]
            v = q.get(j["name"], 0.0)
            if j["type"] in ("revolute", "continuous"):
                M = M @ make_tf(axis_rotation(j["axis"], v))
            elif j["type"] == "prismatic":
                M = M @ make_tf(None, j["axis"] * v)
            T[ch] = M
        return T

    def world_points(self, q_arm, width):
        """(arm_points, finger_points) in the world frame."""
        T = self.frames(q_arm, width)
        arm, fing = [], []
        for link, P in self.points.items():
            if link not in T:
                continue
            W = P @ T[link][:3, :3].T + T[link][:3, 3]
            (fing if link in FINGER_LINKS else arm).append(W)
        return np.vstack(arm), np.vstack(fing)


class Box:
    """Oriented box obstacle."""

    def __init__(self, name, center, R, half):
        self.name = name
        self.c = np.asarray(center, float)
        self.R = np.asarray(R, float)
        self.h = np.asarray(half, float)

    def inside(self, P, margin):
        L = (P - self.c) @ self.R
        return np.all(np.abs(L) <= self.h + margin, axis=1)


def object_box(obj, xyz, R, name=None):
    """Oriented bounding box of an object's collision shapes."""
    pts = []
    for sh in obj.shapes:
        Rs = rpy_to_matrix(*sh.rpy)
        if sh.kind == "box":
            local = _box_points(sh.size, 2)
        else:
            r, h = sh.size
            local = _cyl_points(r, h, 8)
            local = np.vstack([local, [[r, r, h / 2], [-r, -r, -h / 2]]])
        pts.append(local @ Rs.T + np.array(sh.xyz))
    P = np.vstack(pts)
    lo, hi = P.min(axis=0), P.max(axis=0)
    c_local = (lo + hi) / 2
    return Box(name or obj.model, np.asarray(xyz) + R @ c_local, R, (hi - lo) / 2)
