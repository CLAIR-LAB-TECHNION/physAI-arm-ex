"""Executing PDDL actions autonomously: perception + the pick-and-place planner.

    executive.py    pick-up / put-in-basket / put-on-table on demand (closed loop)
    views.py        camera poses of the wrist RealSense and the motions to them
    depth_world.py  the depth seen -> the obstacles of every motion (not the item models)
    obstacles.py    the pick-and-place planner on that, and depth helpers
    drop.py         where to drop an item into the basket

scripts/action_console.py and scripts/goal_console.py run them in-process
(lab_replica/console.py); scripts/skill_server.py serves them over ROS 2
(lab_skills_msgs/action/ExecuteAction), and scripts/do_action.py calls it.
"""
