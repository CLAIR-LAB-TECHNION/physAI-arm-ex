"""State estimation: from the wrist camera's images to where the items are,
and how to grasp one.

The robot takes RGB-D frames (a colour image and an aligned depth image)
with the wrist camera and asks SAM 3 (https://huggingface.co/facebook/sam3),
a segmentation model, for every item by its name. Each answer is a mask (a
boolean image) with a score. The framework (perception/scene.py) turns them
into the scene with these functions:

    OBSERVATION_JOINTS   the arm's joints from which the camera sees the whole workspace
    SIDE_VIEW            a camera pose looking at the workspace at an angle
    close_views          camera poses looking straight down at a spot from closer
    pixels_to_world      depth pixels -> 3D points in the world frame
    estimate_placement   one item's points and those around it -> where and how it rests
                         (height_map, pose_cost, fit_pose: its model matched to the points)
    assign_masks         the candidate masks of one frame -> at most one per item
    fuse_views           the detections of several frames -> one per item
    grasp_in_image       an item's mask -> top-down grasps in the image
    grasp_pose           a grasp in the image -> the gripper's 3D pose and opening

Conventions (see lab_replica/toolkit.py for the types):
  * The world frame has z up, z = 0 on the table top (config/cell.yaml).
  * The camera's optical frame has z along the view, x to the right in the
    image and y down (the ROS convention for cameras,
    https://www.ros.org/reps/rep-0103.html#suffix-frames); a pixel (u, v) is
    column u, row v, so an image array is indexed [v, u].
  * K is the 3x3 intrinsic matrix of the pinhole camera (fx, fy on the
    diagonal, the image centre cx, cy in the last column;
    https://en.wikipedia.org/wiki/Pinhole_camera_model), T_world_cam the
    4x4 pose of the optical frame in the world.
  * A frame (perception/rgbd.py Frame) has .rgb (H, W, 3) uint8, .depth
    (H, W) float32 in metres along the view (z of the optical frame, NaN
    where the camera measured nothing), .K and .T_world_cam.
"""

import math  # noqa: F401

import cv2  # noqa: F401
import numpy as np  # noqa: F401

from ..toolkit import (Estimate, Grasp, ImageGrasp, Placement, down_view, gauss, gripper_pose,  # noqa: F401
                       hand_eye, look_at, mask_overlap, model_height_map, rect_fit, rg6, robust_top, wrap)

# ---- where the camera looks from ----------------------------------------------------------

# The camera looks with robot.survey(view, scene=None, ...): `view` is either
# the arm's six joints or a pose of the camera (4x4, its optical frame in
# the world), or a list of them (the first reachable is used). Given the
# scene of earlier views, survey adds what the new view sees to it: an item
# is recognised from any view, and its position comes from the view that sees
# it best (straight down and close).

# TODO (warm-up): the observation configuration, the arm's six joint angles
# [rad] (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3) from
# which the wrist camera sees the whole workspace. Every action starts by
# looking from here: SAM 3 finds the items and the basket in this one image,
# so the camera must see all of the blue board (0.70 x 0.59 m), with the
# basket anywhere on it, without the gripper or the arm in the way. Too far
# away, the items get small and SAM 3 misses them. Too close, the board's
# edges are out of the image. The arm must also be able to stand here
# holding an item without touching the table or the wall.
#
# Find it on the robot with run/workspace_view.sh real (README, warm-up).
OBSERVATION_JOINTS = None     # TODO

# TODO (task 6): a camera pose (4x4) looking at the workspace from the front
# at an angle. From straight above SAM 3 does not recognise every item (the
# blue cup's yellow stripe is on its side, the spray bottle is a grey head on
# a red blob from above). toolkit.look_at(eye, target) makes the pose.
SIDE_VIEW = None     # TODO

# TODO (task 6): how high above the table the camera is for a closer look [m].
CLOSE_HEIGHT = None


def close_views(x, y, height=CLOSE_HEIGHT):
    """Camera poses (a list of 4x4) straight above (x, y), `height` above
    the table, looking down, for a closer look at an item. robot.survey
    takes the first reachable one.

    Hints: toolkit.down_view(x, y, height, up) makes one. The camera is beside
    the gripper (toolkit.hand_eye()), so turning the camera about the vertical
    (the `up` direction) moves the gripper around the spot, and some turns put
    the arm where it cannot be. Give several turns, the most comfortable for
    the arm first (e.g. those that put the gripper far from the robot's base:
    near the base the arm folds over its own shoulder)."""
    # TODO
    raise NotImplementedError("state_estimation.close_views (exercise/state_estimation.py)")


# ---- from pixels to the world -------------------------------------------------------------


def pixels_to_world(u, v, depth, K, T_world_cam):
    """World points (N, 3) of the pixels (u[i], v[i]) (arrays of length N)
    whose depth along the camera's view (z of the optical frame, metres) is
    depth[i].

    Hints: invert the pinhole projection to get each point in the camera's
    optical frame, then transform it with T_world_cam (a rotation and a
    translation). Work on whole numpy arrays, not pixel by pixel: this runs
    on every pixel of every image."""
    # TODO
    raise NotImplementedError("state_estimation.pixels_to_world (exercise/state_estimation.py)")


# ---- one item: where it rests ---------------------------------------------------------------


def estimate_placement(item, pts, around, table_z, basket=None, basket_xy=None):
    """How `item` rests: an Estimate, or None when there are too few points
    (fewer than about 30) to tell.

    pts        the world points (N, 3) of the item's mask
    around     the world points (M, 3) around the item (its mask, the table
               near it, ...) without those of the other items and the basket
    item       the item's model (objects/*.py): .name, .kind, .spec (its entry
               in config/items.yaml, e.g. spec["stable_poses"], spec["size"] of a
               block), .poses {resting pose name: RestPose} with .top (its height
               above the support) and .footprint (its outline (N, 2) in the
               rested frame)
    table_z    the table's height (world z)
    basket     the basket's model (objects/basket.py: .height, .floor, .r_top,
               .inner_radius(z)), basket_xy its centre (x, y), or None when not
               known (then everything is on the table)

    Returns toolkit.Estimate(placement, plausibility, n_points, top_z, notes):
      placement     toolkit.Placement(item.name, rest, x, y, yaw, support, z_support):
                    the resting pose, the item frame's x, y, its yaw about the
                    vertical, the support ("table" or "basket") and its height
      plausibility  0 to 1: how well the points fit this item. The robot
                    multiplies SAM 3's score by it, so a mask SAM 3 gave the
                    wrong name (a block called "red can") should get a low one
      n_points      len(pts); top_z the height of the item's top seen (world z)

    Hints:
      * Start with the blocks (task 2): a block rests "flat", and from above
        its top is a square. The points near the top of the mask
        (toolkit.robust_top) outline it, toolkit.rect_fit gives its centre and
        direction. A cube looks the same every 90 degrees: any of those yaws
        is right.
      * The plausibility: compare the height of the top above the support,
        and the size of the outline, with the model's (toolkit.gauss turns an
        error into a score).
      * The support is the basket when the points are inside its rim. In the
        basket an item may lie on others, so its support is not the floor.
      * Every item (task 6): match the model, seen from above, to the points
        around it (height_map, pose_cost, fit_pose), for each of its resting
        poses. Use `around`, not only `pts`: where the camera sees the table,
        the item is not; with its mask alone the model could slide and turn
        as long as it covers the mask."""
    # TODO
    raise NotImplementedError("state_estimation.estimate_placement (exercise/state_estimation.py)")


# Task 6: matching the item's model to the points, both seen from above as
# height maps. The model's comes from toolkit.model_height_map(item, rest,
# cell), made from its collision shapes, so it works for every kind of item,
# also those added with new-item:. Suggested functions and constants:

MAP_CELL = None         # TODO: the cell of the model's height map [m] (millimetres)
TRUNCATE = None         # TODO: pose_cost counts a height difference up to this much [m]
BELOW_WEIGHT = None     # TODO: pose_cost's weight where the model stands higher than the points (the
                        # camera sees into a cup or a tape roll, the collision shapes are solid)

# TODO: the search of fit_pose, coarse to fine: per stage (cell of the
# points' height map [m], yaw step [deg], x/y step [m], yaw span [deg],
# x/y span [m], how many poses it keeps for the next stage).
SEARCH = None     # TODO


def height_map(pts, z_support, cell):
    """The points (N, 3) seen from above on a grid of `cell` x `cell`
    squares: (centres (K, 2) of the cells that hold points, height above
    z_support of the highest point in each (K,)). The highest, because from
    above the camera sees an item's top; lower points in a cell are its sides.

    Hint: integer cell indices (np.floor), then one value per cell; sorting
    (np.lexsort) or np.unique with return_inverse avoid a Python loop."""
    # TODO
    raise NotImplementedError("state_estimation.height_map (exercise/state_estimation.py)")


def pose_cost(model, cells, heights, x, y, yaw):
    """How badly the model, at the positions (x[k], y[k]) turned by yaw,
    explains the points' heights (height_map: cells, heights). Returns (K,),
    one cost per position: e.g. the mean squared height difference over the
    cells, robust to the few cells that hold bits of something else
    (TRUNCATE, BELOW_WEIGHT).

    model    toolkit.model_height_map(item, rest): model.at(u, v) are its
             heights at the points (u, v) of the item's rested frame (0 off
             the item)

    Hint: move the cells into the rested frame of each candidate pose (the
    inverse of a rotation by yaw and a translation by (x, y)); numpy
    broadcasting does all K positions at once."""
    # TODO
    raise NotImplementedError("state_estimation.pose_cost (exercise/state_estimation.py)")


def fit_pose(item, rest, around, z_support, centre):
    """(cost, x, y, yaw): the pose of `item` resting as `rest` on a support
    at z_support whose model best explains the heights of the points
    `around` it, near `centre` (the middle of its mask).

    Hint: try many poses (pose_cost), coarse to fine (SEARCH): all yaws and a
    few centimetres around `centre` on a coarse grid, then finer grids around
    the best few. Symmetric items (a cube, a can standing) fit equally well at
    several yaws: any of them is right."""
    # TODO
    raise NotImplementedError("state_estimation.fit_pose (exercise/state_estimation.py)")


# ---- which mask is which item -------------------------------------------------------------


def assign_masks(candidates, min_score, max_overlap=0.5):
    """At most one mask per item and one item per mask, from the candidates
    of one frame (a list of toolkit.Detection: .item, .score (SAM 3's score x
    the plausibility), .mask, ...). A name may match several objects, and one
    object several names. Candidates scoring below min_score are dropped.
    Two masks are the same object when they overlap by more than max_overlap
    (toolkit.mask_overlap). Returns {item: Detection}.

    Hint: the best candidates should choose first."""
    # TODO
    raise NotImplementedError("state_estimation.assign_masks (exercise/state_estimation.py)")


def fuse_views(per_view, weights):
    """One detection per item from several frames: per_view[k] is the
    {item: Detection} of frame k, weights[k] how much that frame counts.
    Returns {item: Detection}.

    Per item, the detection with the best score x weight, but two items
    cannot stand in one place (on the same support, a couple of centimetres
    apart): when an item's best detection is where another item already is,
    use its next best one, or none. d.estimate.placement says where a
    detection puts its item.

    Hint: as in assign_masks, the items with the best detections choose first."""
    # TODO
    raise NotImplementedError("state_estimation.fuse_views (exercise/state_estimation.py)")


# ---- grasping ---------------------------------------------------------------------------

GRASP_BAND = None       # TODO: the top of an item is its points this far below its highest [m]
ROUND_ASPECT = None     # TODO (task 6): a top this much longer than wide is elongated
SQUARE_FILL = None      # TODO (task 6): a top that fills this much of its rectangle is square
                        # (a disk fills pi / 4 of its square)


def grasp_in_image(frame, mask):
    """Top-down grasps of the item whose mask this is, in the image: each
    the pixel the fingers close around and the direction they close along.
    A list of toolkit.ImageGrasp(u, v, angle, width_px), best first ([]
    without enough depth in the mask). `angle` is in the image, in radians
    from +u towards +v.

    frame  the frame (.rgb, .depth, .K, .T_world_cam), taken looking straight down
    mask   the item's boolean mask in that frame

    Hints:
      * The fingers close on the top of the item: the pixels of the mask
        that are nearest to the camera.
      * Start with the blocks (task 2): close across a pair of opposite
        faces. cv2.minAreaRect (OpenCV,
        https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
        fits a rectangle to pixels: mind its angle convention.
      * Other shapes (task 6): an elongated top (a can lying down) across
        its narrow side; a round one (a cup, a tape roll) from any side, so
        offer several directions. Mind the mug's handle."""
    # TODO
    raise NotImplementedError("state_estimation.grasp_in_image (exercise/state_estimation.py)")


TIP_CLEARANCE = None    # TODO: the fingertips stay this far above the table [m]
KNUCKLE_MARGIN = None   # TODO: the item's top stays this far below the knuckles [m]


def grasp_pose(frame, image_grasp, mask, table_z):
    """The 3D grasp for a grasp in the image: a toolkit.Grasp(pose, width,
    tips_z, top_z) with
      pose    the 4x4 world pose of the gripper's mounting face
              (rg6_base_link) at the grasp: toolkit.gripper_pose(x, y, z, yaw)
              points the gripper straight down, its fingers closing along its
              y axis, so yaw is the closing direction minus 90 degrees
      width   the item's width between the pads [m]
      tips_z  the fingertips' height at the grasp (world z)
      top_z   the item's top (world z)

    Hints:
      * pixels_to_world takes pixels back into the world: the grasp's pixel,
        and pixels on either side of it along the closing direction, give the
        grasp's centre and its closing direction in the world.
      * The width: how far the item's 3D points extend along the closing
        direction (percentiles ignore a stray pixel or two).
      * How deep: the RG6's knuckles reach into the space between the pads
        rg6.free_height(width) above the fingertips, so the item's top must
        stay below that; deeper is a firmer grip, but the fingertips may not
        touch the table (TIP_CLEARANCE).
      * The mounting face is rg6.depth_from_width(width) above the fingertips
        (the RG6's fingers move on a parallelogram: the narrower, the deeper)."""
    # TODO
    raise NotImplementedError("state_estimation.grasp_pose (exercise/state_estimation.py)")
