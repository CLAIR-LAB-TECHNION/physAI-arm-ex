"""Camera poses of the wrist RealSense and the arm motions to reach them.

A view is the camera at `eye` looking at `target`, with the world direction
`up` pointing up in the image. The arm pose for it follows from the
hand-eye transform (rg6_base_link -> colour optical frame, from the robot
description), by inverse kinematics.

The actions move to views with the exercise's plan_view (MoveIt); the
ViewPlanner here (the earlier collision-checked motions) is still used by
tools/calibrate_wrist_camera.py.
"""

import math

import numpy as np

from ..planning.motion import PlanError, gripper_pose
from ..planning.skills import TRANSIT_Z, TRANSIT_Z_MAX

VIEW_TRANSIT_Z_MAX = 0.62   # highest transit on the way to a camera view
from ..robot.kinematics import Chain, robot_urdf
from ..transforms import inv_tf, make_tf

OPTICAL_FRAME = "wrist_camera_color_optical_frame"


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """World pose of a camera optical frame (z along the view, x right, y down)."""
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    if np.linalg.norm(x) < 1e-6:
        raise ValueError("view direction parallel to `up`")
    x /= np.linalg.norm(x)
    return make_tf(np.c_[x, np.cross(z, x), z], eye)


def view_from_config(v):
    return look_at(v["eye"], v["target"], v.get("up", (0, 0, 1)))


def down_view(x, y, height, up=(0.0, -1.0, 0.0)):
    """Camera straight above (x, y), `height` above the table, looking down."""
    return look_at((x, y, height), (x, y, 0.0), up)


def hand_eye(ur_type):
    """The camera's optical frame in the frame of rg6_base_link (4x4), from
    the robot description (cell.yaml wrist_camera)."""
    urdf = robot_urdf(ur_type)
    q = np.zeros(6)
    return inv_tf(Chain(urdf, "world", "rg6_base_link").fk(q)) @ Chain(urdf, "world", OPTICAL_FRAME).fk(q)


class ViewPlanner:
    """Arm configurations and collision-checked motions for camera views.
    `skills` is a planning.skills.Skills (its planner, collision checks and
    transit motions are reused)."""

    def __init__(self, skills, ur_type):
        self.sk = skills
        self.P = skills.P
        urdf = robot_urdf(ur_type)
        grip = Chain(urdf, "world", "rg6_base_link")
        cam = Chain(urdf, "world", OPTICAL_FRAME)
        q = np.zeros(6)
        self.T_grip_cam = inv_tf(grip.fk(q)) @ cam.fk(q)

    def camera_pose(self, q):
        return self.P.kin.fk(q) @ self.T_grip_cam

    def ik_view(self, T_cam, q_ref):
        """Arm joints that put the camera at T_cam (nearest to q_ref), or None."""
        return self.P.ik(T_cam @ inv_tf(self.T_grip_cam), q_ref)

    def above_held(self, T_cam, held, q_ref, z_clear, max_raise=0.06):
        """T_cam raised (1 cm steps, at most max_raise) until the held item
        hangs above z_clear at the view, if the arm reaches that; else T_cam.
        Under a tilted gripper a long item swings out, possibly over space
        the camera will not see (bounded only by max_item_height), and the
        way back to a level gripper must keep clear of it."""
        for dz in np.arange(0.0, max_raise + 1e-6, 0.01):
            T = T_cam.copy()
            T[2, 3] += dz
            q = self.ik_view(T, q_ref)
            if q is None:
                break
            box = self.sk.held_box(held, self.P.kin.fk(q))
            corners = box.c + (np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
                               * box.h) @ box.R.T
            if corners[:, 2].min() >= z_clear:
                return T
        return T_cam

    def gripper_tilt(self, q):
        """Angle [rad] between the gripper axis and straight down."""
        return math.acos(max(-1.0, min(1.0, -self.P.kin.fk(q)[2, 2])))

    def _rise(self, q0, z):
        """Straight up to height z of the mounting face (if lower), keeping the orientation."""
        T = self.P.kin.fk(q0)
        if T[2, 3] >= z - 1e-3:
            return [np.asarray(q0, float)]
        path = [np.asarray(q0, float)]
        n = int(math.ceil((z - T[2, 3]) / 0.01))
        for i in range(1, n + 1):
            Ti = T.copy()
            Ti[2, 3] = T[2, 3] + (z - T[2, 3]) * i / n
            q = self.P.ik(Ti, path[-1], max_jump=0.3)
            if q is None:
                raise PlanError("cannot rise straight up")
            path.append(q)
        return path

    def move_to_view(self, q0, T_cam, world, width, held=None, label="look"):
        """Collision-checked joint path from q0 to the view. Raises PlanError.

        First the way the pick-and-place motions travel: gripper level, rise,
        arc around the base at a safe height to above the view, then tilt to
        the view; else one joint-space move (rising first if low)."""
        T_g = T_cam @ inv_tf(self.T_grip_cam)
        x = T_g[:3, 0]
        psi = math.atan2(x[1], x[0])
        err = None
        try:
            path = self.to_topdown(q0, world, width, held) if self.gripper_tilt(q0) > math.radians(2) else [q0]
            z = min(T_g[2, 3], VIEW_TRANSIT_Z_MAX)
            arc, _ = self.sk._transit(path[-1], T_g[:2, 3], psi, z, width, world, held=held, label=label)
            q_view = self.ik_view(T_cam, arc[-1])
            if q_view is None:
                raise PlanError(f"{label}: camera pose out of reach")
            tail = self.P.joint_move(arc[-1], q_view, step=0.03)
            self.sk.check_path(tail, width, world, held=held, label=label)
            return path + arc[1:] + tail[1:]
        except PlanError as e:
            err = e
        rise = self._rise(q0, TRANSIT_Z)
        q_view = self.ik_view(T_cam, rise[-1])
        if q_view is None:
            raise PlanError(f"{label}: camera pose out of reach")
        path = rise + self.P.joint_move(rise[-1], q_view, step=0.03)[1:]
        try:
            self.sk.check_path(path, width, world, held=held, label=label)
        except PlanError as e:
            raise PlanError(f"{err}; {e}")
        return path

    def to_topdown(self, q0, world, width, held=None):
        """From a view back to a top-down gripper pose at the same place (the
        skills' motions start from such poses): rise if needed, then level.
        A high view (the survey, raised further over a held item) can be at
        the edge of the arm's reach, where no level pose exists at its
        height: then lower heights are tried too."""
        T = self.P.kin.fk(q0)
        x = T[:3, 0]
        psi = math.atan2(x[1], x[0])
        err = None
        z0 = max(T[2, 3], TRANSIT_Z)
        higher = [z for z in np.arange(TRANSIT_Z, TRANSIT_Z_MAX + 1e-6, 0.04) if z > z0 + 0.01]
        lower = list(np.arange(z0 - 0.02, TRANSIT_Z - 1e-6, -0.02))
        for z in [z0] + higher + lower:
            try:
                rise = self._rise(q0, z)
                q = self.P.ik(gripper_pose(T[0, 3], T[1, 3], z, psi), rise[-1])
                if q is None:
                    raise PlanError("no top-down pose near the view")
                path = rise + self.P.joint_move(rise[-1], q, step=0.03)[1:]
                self.sk.check_path(path, width, world, held=held, label="to top-down")
                return path
            except PlanError as e:
                err = e
        raise err
