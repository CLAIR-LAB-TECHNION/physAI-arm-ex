"""The skills: the PDDL actions of the domain, performed by the robot.

The action (name ?x ...) is performed by the function of the same name with
'-' replaced by '_', given the robot and the action's parameters:

    (pick-up red-block)        ->  pick_up(robot, "red-block")
    (put-on-table red-block)   ->  put_on_table(robot, "red-block")
    (put-in-basket red-block)  ->  put_in_basket(robot, "red-block")

A skill returns a message saying what it did, or raises ActionError (from
the toolkit: the action cannot be performed) or PlanError (no motion found).
It checks the action's preconditions on the gripper, looks at the scene,
decides with the state estimation (state_estimation.py) and the motion
planning (motion_planning.py), moves, and checks the action's effects on
the gripper. The robot does the rest by itself: after a skill succeeds it
returns home (motion_planning.plan_home); after one fails having moved the
arm (robot.run) with nothing left in the gripper, it opens the gripper and
rises.

The robot (autonomy/executive.py):
    robot.items, robot.basket       {name: item model}, the basket's model
    robot.cell, robot.cfg           config/cell.yaml, config/perception.yaml (as dicts)
    robot.table_z, robot.force      the table's height (world z), the grip force [N]
    robot.mv                        MoveIt (moveit_client.MoveIt), for motion_planning.py
    robot.held                      what the gripper holds as far as the skills know: None, or
                                    .item, .held (toolkit.Held), .source (the Placement it was
                                    picked up from, or None), .assumed (not seen being picked up)
    robot.log(text)                 a line in the log
    robot.phase(text)               the current phase (in the log and the camera window)
    robot.holding()                 the gripper detects a grip (a fresh reading of its sensor)
    robot.width()                   the gripper's opening now [m]
    robot.current()                 the arm's joints now
    robot.wait(seconds)             let the robot's state come in for a while
    robot.survey(view, scene=None, held=None, exclude=())
                                    move the camera to `view` (the arm's joints, a 4x4 camera pose,
                                    or a list of them: the first reachable; state_estimation.py)
                                    and recognise what it sees, added to `scene` when given.
                                    `held`: the held item (Held), kept clear of on the way;
                                    `exclude`: items not to look for (the held one). Returns a
                                    Scene with .detections {item: Detection}, .basket_xy, .frames
                                    (the last is this view's), .depth, .obstacles. Raises PlanError
                                    when no view is reachable
    robot.best_frame(scene, item=None)
                                    the scene's frame looking straight down from nearest, or the one
                                    `item`'s estimate comes from
    robot.grasp_mask(frame, scene, item)
                                    the pixels of `item` in `frame` to propose a grasp on (its mask
                                    joined with its model's outline, without its neighbours)
    robot.set_obstacles(scene, exclude=())
                                    MoveIt's planning scene: the table, the wall and what `scene`
                                    saw (motion_planning.scene_obstacles), without the items in
                                    `exclude`. Call it before planning
    robot.table_region(basket_xy, depth=None)
                                    where items may be put on the table (for find_table_spot);
                                    pass scene.depth
    robot.drop_spots(scene, held, yaws)
                                    the basket's drop spots for the held item with the item yaws
                                    `yaws`: .spots best first, each (cost, x, y, yaw, z_pile: the
                                    top of what is piled there, world z), and .z_floor
    robot.run(steps)                perform a plan's steps (raises ActionError when one fails)
    robot.now_holding(held, source=None, assumed=False), robot.now_empty()
                                    what the gripper holds from now on (survives restarts)
    robot.placed(item, placement)   an item just put down (kept clear of on the way home)
    robot.show(frame, draw, title)  a picture in the camera window (the toolkit's show_* draw the
                                    usual ones: show_image_grasps, show_grasp, show_table_spot,
                                    show_drop)
After every look the robot shows by itself what it recognised.
"""

import math  # noqa: F401

from ..toolkit import (ActionError, Held, PlanError, Placement, rg6, show_drop, show_grasp,  # noqa: F401
                       show_image_grasps, show_table_spot, spread, wrap)
from . import motion_planning as mp  # noqa: F401
from . import state_estimation as se  # noqa: F401

DROP_CLEARANCE = None    # TODO: dropping into the basket: the item's bottom this far above the rim or the pile [m]
RECOGNISED = None        # TODO (task 6): an item found with a lower score (SAM 3 x plausibility) is looked
                         # at again from other views


def pick_up(robot, item):
    """(pick-up item): find `item`, grasp it from above, lift it. Returns a
    message such as "holding red-block".

    Hints:
      * Precondition: the gripper is empty (robot.holding()). Refuse with
        ActionError otherwise, before moving anything.
      * Look from the observation configuration
        (robot.survey(se.OBSERVATION_JOINTS)) and find `item` in the scene's
        detections.
      * Grasps: in the frame the item's estimate comes from
        (robot.best_frame(scene, item)), on its pixels (robot.grasp_mask),
        proposed in the image (se.grasp_in_image) and taken to the world
        (se.grasp_pose). Drop the ones too wide for the RG6 (rg6.MAX_WIDTH).
      * Plan with the item itself not an obstacle
        (robot.set_obstacles(scene, exclude=(item,))), then mp.plan_pick, and
        run the steps (robot.run).
      * Effect: check that the gripper holds it (robot.holding(), robot.width()),
        and tell the robot how it holds it: robot.now_holding(toolkit.Held(...),
        source=the item's Placement). The put skills need the Held: the
        grasp's centre in the item's rested frame (offset_obj) and the item's
        yaw relative to the gripper's (yaw_rel), from the item's Placement and
        the gripper pose plan_pick used.
      * Show what you decided (show_image_grasps, show_grasp): it helps
        when something goes wrong.
      * Task 6: when the item is not recognised well from above (score below
        RECOGNISED), look from other views too (se.SIDE_VIEW, passing the
        scene to add to), and from closer (se.close_views) before grasping."""
    # TODO
    raise NotImplementedError("skills.pick_up (exercise/skills.py)")


def put_on_table(robot, item, target_xy=None):
    """(put-on-table item): put the held `item` down on a free spot of the
    table, the one nearest to where it was picked up (robot.held.source), or
    to target_xy (x, y) when given. Returns a message.

    Hints:
      * Precondition: the gripper holds something. robot.held says how it
        holds `item`. It is None when the robot did not see it being picked
        up (e.g. the console was restarted): then assume a grasp of the item's
        model (robot.items[item].poses[rest].grasps, objects/base.py Grasp) or
        refuse.
      * Look (robot.survey with held= and exclude=), set the obstacles, then
        mp.find_table_spot. It needs the region (robot.table_region), the
        footprints of what stands on the table (the other detections'
        model.footprint_world(...), the scene's obstacles' .polygon) and
        the held item's box (mp.held_box).
      * Effect: after robot.run, check that the gripper let go, then
        robot.now_empty() and robot.placed(item, placement)."""
    # TODO
    raise NotImplementedError("skills.put_on_table (exercise/skills.py)")


def put_in_basket(robot, item):
    """(put-in-basket item): drop the held `item` into the basket, from just
    above its rim, over its emptiest spot (it may land on other items).
    Returns a message.

    Hints:
      * Precondition and effect as in put_on_table.
      * robot.drop_spots(scene, held, yaws) rates spots in the basket for a
        few item yaws (say every 45 degrees). Release the item DROP_CLEARANCE
        above the rim, or above the pile there if that is higher
        (mp.plan_place with approach=mp.DROP_APPROACH). The basket's rim is
        robot.basket.height above the table, its floor robot.basket.floor.
      * The best spot may be out of reach: try the next ones
        (toolkit.spread picks a few, apart)."""
    # TODO
    raise NotImplementedError("skills.put_in_basket (exercise/skills.py)")
