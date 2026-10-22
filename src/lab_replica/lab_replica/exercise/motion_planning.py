"""Motion planning: the robot's motions for the camera views and the three
PDDL actions, planned with MoveIt.

MoveIt (https://moveit.picknik.ai/humble/doc/concepts/concepts.html) is the
motion planning framework of ROS. It runs as a separate program, move_group
(https://moveit.picknik.ai/humble/doc/concepts/move_group.html), which the
consoles start. You never talk to it directly: `moveit` below is a
lab_replica.moveit_client.MoveIt, which hides ROS and takes and returns
numpy values. It plans collision-free with respect to its planning scene
(https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html):
the robot itself, the table and the wall, plus the obstacles of
scene_obstacles.

    moveit.plan_to_joints(q_start, q_goal, width, held=None)       -> joint path
          a collision-free path found by a sampling-based planner, OMPL's
          RRT-Connect (https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html):
          it finds a way around obstacles, but not always a short or smooth one
    moveit.plan_to_pose(q_start, T_goal, width, held=None)          -> joint path
          the same to any configuration that puts the gripper at T_goal
    moveit.cartesian_path(q_start, [T, ...], width, held=None, min_fraction=1.0)
                                                                    -> joint path
          the gripper along straight lines through the poses
          (https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html#cartesian-paths);
          raises PlanError when it gets less than min_fraction of the way
    moveit.is_valid(q, width, held=None)                            -> bool: touches nothing
    moveit.contacts(q, width, held=None)                            -> what touches what there

A joint path is a list of arrays of the six arm joints (shoulder_pan,
shoulder_lift, elbow, wrist_1, wrist_2, wrist_3) [rad], from q_start. Poses T
are 4x4 world poses of rg6_base_link, the RG6's mounting face; `width` is
the gripper's opening [m] (the fingers are part of the robot), `held` the box
of the item in the gripper in the gripper's frame (held_box), which then
moves with it. Everything raises toolkit.PlanError when no motion is found.

A plan for the robot is a list of steps, which the robot runs in order:
    toolkit.trajectory(label, path, slow=False, slow_last=None, slow_first=None)
          move the arm along a joint path (timed for you); slow_last /
          slow_first: the last / first so many metres slowly (a descent, a lift)
    toolkit.gripper_step(label, width, force, expect_grip=False, drop=None)
          move the fingers to `width` with at most `force` [N]; expect_grip:
          the step fails unless the fingers stop on something

The robot's kinematics (toolkit, no MoveIt needed):
    fk(q)          the world pose of rg6_base_link at the joints q
    ik_all(T)      every joint solution (each angle in [-pi, pi]) that puts it at T
                   (https://moveit.picknik.ai/humble/doc/concepts/kinematics.html)
    joint_limits() {joint name: (lower, upper)}; ARM_JOINTS the joint names in order

Conventions: the gripper points straight down for grasping and placing;
its yaw is the direction of its x axis in the world, the fingers close
along its y axis (toolkit.gripper_pose(x, y, z, yaw)). The fingertips are
rg6.depth_from_width(width) below the mounting face.
"""

import math  # noqa: F401

import numpy as np  # noqa: F401

from ..toolkit import (ARM_JOINTS, Box, Cylinder, PlanError, Placement, finger_footprints, fk,  # noqa: F401
                       gripper_pose, gripper_step, ik_all, joint_limits, object_box, placement_pose,
                       polygon_inside_rect, polygons_intersect, rg6, rpy_to_matrix, trajectory, wrap)

# The constants of the motions [m]. Choose them, then try them on the robot.
OPEN_MARGIN = None          # TODO: open this much wider than the item on each side before closing
SQUEEZE = None              # TODO: close this much below the item's width (the force limits the grip)
APPROACH = None             # TODO: the gripper comes down to a grasp (and leaves) along this last straight stretch
PLACE_APPROACH = None       # TODO: ... and to a place on the table
DROP_APPROACH = None        # TODO: ... and to a drop into the basket (a tall item is dropped from high up)
RELEASE_GAP = None          # TODO: an item put on the table is released this far above it
SLOW = None                 # TODO: the last (first) part of a descent (lift) that is slow
RISE_Z = None               # TODO: plan_path: the height the mounting face rises to before a straight line
JOINT_WEIGHTS = None        # TODO: (6,) how much each joint's change counts in choose_solution


# ---- inverse kinematics and frames ----------------------------------------------------------


def choose_solution(solutions, q_ref):
    """Of the inverse kinematics solutions (toolkit.ik_all: arrays of six
    angles in [-pi, pi]), the one nearest to q_ref, or None.

    Hints: a joint angle and the same angle plus or minus 2 pi put the arm
    in the same place, and the joints may turn further than pi
    (toolkit.joint_limits): for each joint use the equivalent angle within
    its limits nearest to q_ref. A solution with a joint that cannot reach
    any equivalent angle is no solution. Measure the distance so that the
    arm moves least (JOINT_WEIGHTS: turning the last wrist joint is harmless)."""
    # TODO
    raise NotImplementedError("motion_planning.choose_solution (exercise/motion_planning.py)")


def camera_to_gripper(T_world_cam, T_grip_cam):
    """The pose of rg6_base_link (4x4) that puts the camera's optical frame at
    T_world_cam; T_grip_cam is the camera's pose in the gripper's frame
    (toolkit.hand_eye()).

    Hint: poses compose by matrix products, T_world_cam = T_world_grip @ T_grip_cam
    (https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)."""
    # TODO
    raise NotImplementedError("motion_planning.camera_to_gripper (exercise/motion_planning.py)")


def held_box(held, item):
    """The box of a held item in the gripper's frame, for MoveIt to move with
    the gripper (a toolkit.Box: centre c, rotation R, half sizes h).

    held   how it is held (toolkit.Held): its resting pose `rest`, `width`
           between the pads, `tip_z` the fingertips' height above its support
           when grasped, `offset_obj` the grasp centre (x, y) in the item's
           rested frame, `yaw_rel` the item's yaw minus the gripper's
    item   the item's model: item.poses[rest] (.rpy, .z: how the item frame
           sits on its support)

    Hints: it was grasped from above. Find the item frame's position and
    rotation in the gripper's frame (z points down from the mounting face,
    the fingertips are rg6.depth_from_width(width) below it), then
    toolkit.object_box(item, xyz, R, name) gives its box. Carried by the arm
    at the grasp (toolkit.box_in_world), the box must be where the item
    stood."""
    # TODO
    raise NotImplementedError("motion_planning.held_box (exercise/motion_planning.py)")


# ---- moving -----------------------------------------------------------------------------


def lifted(T, dz):
    """The pose T (4x4) moved up by dz [m] (a new array)."""
    # TODO
    raise NotImplementedError("motion_planning.lifted (exercise/motion_planning.py)")


def turned(T):
    """The same gripper pose turned half a turn about its own z axis: the RG6
    is symmetric, so it grasps the same way."""
    # TODO
    raise NotImplementedError("motion_planning.turned (exercise/motion_planning.py)")


def joint_line(q0, q1, step=0.02):
    """Configurations along the straight line in joint space from q0 to q1
    (both included), no joint moving more than `step` [rad] between two of
    them."""
    # TODO
    raise NotImplementedError("motion_planning.joint_line (exercise/motion_planning.py)")


def straight_path(moveit, q0, q1, width, held=None):
    """The straight line in joint space from q0 to q1 (joint_line) when no
    configuration on it touches anything (moveit.is_valid), else None. The
    shortest way when nothing is in it."""
    # TODO
    raise NotImplementedError("motion_planning.straight_path (exercise/motion_planning.py)")


def plan_path(moveit, q0, q1, width, held=None):
    """A collision-free joint path from q0 to q1, the simplest that is free.

    Hint: MoveIt's planner (moveit.plan_to_joints) always finds a way if there
    is one, but it can be a long detour. Try simpler paths first: the
    straight line in joint space, or straight up first (a Cartesian path to
    RISE_Z) and then the straight line."""
    # TODO
    raise NotImplementedError("motion_planning.plan_path (exercise/motion_planning.py)")


def move_to_pose(moveit, q0, T, width, held=None):
    """A collision-free joint path from q0 to the gripper pose T.

    Hints: the goal is an inverse kinematics solution (toolkit.ik_all,
    choose_solution): the one nearest to q0 keeps the arm's elbow and wrist
    the way they are. When it is in collision (moveit.is_valid), or no path
    reaches it, try the others. Raise PlanError at once when T is out of
    reach (no solution), and otherwise say why nothing worked
    (moveit.contacts names what the robot would touch)."""
    # TODO
    raise NotImplementedError("motion_planning.move_to_pose (exercise/motion_planning.py)")


def plan_view(moveit, q0, T_cam, T_grip_cam, width, held=None, label="look"):
    """The steps (a list of toolkit.trajectory / gripper_step) that move the
    camera to the camera pose T_cam (4x4, its optical frame in the world).
    T_grip_cam is the camera's pose in the gripper's frame. robot.survey
    calls it for a view given as a camera pose."""
    # TODO
    raise NotImplementedError("motion_planning.plan_view (exercise/motion_planning.py)")


def plan_observation(moveit, q0, q_obs, width, held=None, label="look"):
    """The steps that move the arm to the configuration q_obs a survey looks
    from (state_estimation.OBSERVATION_JOINTS, ...). robot.survey calls it for
    a view given as joints. Raise PlanError saying what the arm would touch
    there (moveit.contacts) when q_obs itself is in collision."""
    # TODO
    raise NotImplementedError("motion_planning.plan_observation (exercise/motion_planning.py)")


def plan_home(moveit, q0, q_home, width, held=None):
    """The steps back to the home configuration q_home. The robot calls it
    after every action that succeeded.

    Hint: what the gripper holds may touch the table at home (a tall item
    hangs low): then go to the home pose raised a little instead."""
    # TODO
    raise NotImplementedError("motion_planning.plan_home (exercise/motion_planning.py)")


def plan_pick(moveit, q0, grasps, force):
    """The steps of a top-down grasp: to above the grasp, open, straight down,
    close, straight up.

    grasps  toolkit.Grasp (the pose of the mounting face at the grasp, the
            item's width, ...), best first: the first one that can be planned
            is taken
    force   the grip force [N]

    Returns (steps, q_end, the grasp taken, the gripper pose used).

    Hints:
      * The way down and up must be straight (moveit.cartesian_path), so the
        fingers do not sweep into the item or its neighbours. The gripper
        moves open on the way down (it is part of the robot model, its width
        matters) and the up path holds the item.
      * The gripper is symmetric: each grasp also works turned half a turn
        (turned), which may be easier for the arm.
      * Close with gripper_step(..., expect_grip=True): the step fails if the
        fingers close on nothing. Close below the item's width (SQUEEZE):
        the force stops the fingers.
      * The RG6's fingers move on a parallelogram: they go deeper as they
        close. Go down to the grasp pose of the item's width and stay there
        while they close."""
    # TODO
    raise NotImplementedError("motion_planning.plan_pick (exercise/motion_planning.py)")


def gripper_for(held, x, y, yaw):
    """The gripper's (x, y, yaw) that puts the held item (toolkit.Held) at
    (x, y, yaw): the inverse of how it was grasped (held.offset_obj,
    held.yaw_rel)."""
    # TODO
    raise NotImplementedError("motion_planning.gripper_for (exercise/motion_planning.py)")


def plan_place(moveit, q0, held, box, x, y, yaw, z_support, gap, force, approach=PLACE_APPROACH, label="lower"):
    """The steps that put the held item down at (x, y, yaw), its bottom `gap`
    above z_support (the height of what it will rest on): carry it to
    `approach` above there, straight down, open, straight up. `box` is its
    held_box, which every motion with the item must carry.

    Hints:
      * The gripper's pose follows from gripper_for and the height of the
        fingertips above the item's bottom when it was grasped (held.tip_z).
      * Open with gripper_step(..., drop=gap) so the robot knows the item
        falls that far.
      * At the edge of the arm's reach (a tall item held high over the far
        side of the basket) the pose above may be out of reach: carry the
        item to the release pose directly then."""
    # TODO
    raise NotImplementedError("motion_planning.plan_place (exercise/motion_planning.py)")


# ---- where to put an item down ------------------------------------------------------------


def sample_spots(region, prefer, step=0.01):
    """Candidate centres (x, y) on a grid of `step` over the region's
    x_range and y_range, nearest to `prefer` (x, y) first."""
    # TODO
    raise NotImplementedError("motion_planning.sample_spots (exercise/motion_planning.py)")


def is_free(footprint, region, obstacles, margin):
    """Whether an item with this footprint (a polygon (N, 2) in the world)
    may be put there: inside the region's x_range and y_range, outside its
    keepout circles region["keepout"] = [(x, y, r), ...], and more than
    `margin` away from every obstacle (polygons (M, 2)).

    Useful: toolkit.polygon_inside_rect, toolkit.polygons_intersect."""
    # TODO
    raise NotImplementedError("motion_planning.is_free (exercise/motion_planning.py)")


def find_table_spot(moveit, q0, held, item, box, region, obstacles, prefer, table_z, force, margin=0.02,
                    max_tries=80):
    """Put the held item on the table: the free spot nearest to `prefer`
    that the robot can reach. Returns (toolkit.Placement of the item there,
    the steps of plan_place). Raises PlanError when there is none.

    held       how it is held (toolkit.Held); item its model, box its held_box
    region     robot.table_region(...): x_range, y_range, keepout, and
               accept(footprint, top z) -> bool (only where the camera can see
               the item again)
    obstacles  the footprints (polygons) of what is on the table

    Hints: try spots (sample_spots) and a few yaws of the item; the item's
    footprint there is item.footprint_world(held.rest, x, y, yaw). The open
    fingers need room too (toolkit.finger_footprints). Planning a motion is
    slow: check everything cheap first, and plan at most max_tries."""
    # TODO
    raise NotImplementedError("motion_planning.find_table_spot (exercise/motion_planning.py)")


# ---- what MoveIt keeps clear of -------------------------------------------------------------

ITEM_PAD = None             # TODO: the items' boxes are this much larger on every side [m]
BASKET_PAD = None           # TODO: ... and the basket's


def scene_obstacles(detections, items, basket, basket_xy, table_z, unknown=(), exclude=()):
    """The obstacles of MoveIt's planning scene besides the table and the wall,
    in the world frame: a list of toolkit.Box(name, centre, R, half sizes)
    and toolkit.Cylinder(name, centre, radius, half height). The robot sets
    them (robot.set_obstacles) before planning.

    detections  {item: Detection}: d.estimate.placement says where it rests
    items       {item: model}; toolkit.placement_pose(model, placement) gives
                its frame (xyz, R), toolkit.object_box(model, xyz, R, name) its box
    basket      the basket's model (.r_top, .height), basket_xy its centre (x, y)
    unknown     what the depth camera saw that no item explains: each with
                .polygon (N, 2), .z_top. Also space no view saw (behind a tall
                item: maybe an item nobody recognised)
    exclude     items left out (the one to pick, the one held)

    Hint: an obstacle slightly too large costs little; one too small, a
    collision."""
    # TODO
    raise NotImplementedError("motion_planning.scene_obstacles (exercise/motion_planning.py)")
