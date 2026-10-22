"""Where an item rests: on which support, in which resting pose, at (x, y,
yaw); and how the gripper holds an item."""

import math
from dataclasses import asdict, dataclass

import numpy as np

from ..transforms import matrix_to_quat, quat_to_matrix, rpy_to_matrix


@dataclass
class Placement:
    item: str                # PDDL name
    rest: str                # resting pose name (see lab_replica/objects)
    x: float
    y: float
    yaw: float               # rotation about the vertical [rad]
    support: str = "table"   # 'table' or 'basket'
    z_support: float = 0.0   # height of the supporting surface (world z)

    def as_dict(self):
        return asdict(self)


@dataclass
class Held:
    """How the gripper holds an item: the item stands in resting pose `rest`
    (its height `top`), the pads `width` apart, the fingertips `tip_z` above
    the item's support at the grasp; the grasp centre at `offset_obj` (x, y)
    in the item's rested frame, the item's yaw minus the gripper's `yaw_rel`."""
    item: str
    rest: str
    width: float          # object width between the pads
    tip_z: float          # fingertip height above the object's support at grasp
    offset_obj: tuple     # grasp centre in the rested frame (x, y)
    yaw_rel: float        # object yaw minus gripper yaw
    top: float            # object height in this resting pose


def placement_pose(item_model, p):
    """World position and rotation matrix of the item frame for a placement."""
    rest = item_model.poses[p.rest]
    R = rpy_to_matrix(0, 0, p.yaw) @ rpy_to_matrix(*rest.rpy)
    xyz = np.array([p.x, p.y, p.z_support + rest.z])
    return xyz, R


def placement_at(item_model, p, xyz, quat):
    """Placement p moved to the pose (xyz, quat_xyzw), e.g. where the item
    settled in Gazebo: x, y, yaw and the support height follow the pose."""
    R = quat_to_matrix(quat)
    rest = item_model.poses[p.rest]
    Rz = R @ rpy_to_matrix(*rest.rpy).T
    yaw = math.atan2(Rz[1, 0], Rz[0, 0])
    return Placement(p.item, p.rest, float(xyz[0]), float(xyz[1]), float(yaw), p.support, float(xyz[2] - rest.z))


def pose_dict(xyz, R):
    """{'xyz', 'quat_xyzw'} for layout.yaml."""
    q = matrix_to_quat(R)
    return {"xyz": [round(float(v), 5) for v in xyz], "quat_xyzw": [round(float(v), 6) for v in q]}
