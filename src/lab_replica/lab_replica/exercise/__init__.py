"""The exercise: task planning, state estimation and motion planning for the
UR5e + RG6 + wrist RealSense cell. Nothing in here touches ROS.

    task_planning/domain.pddl   the PDDL domain (the actions' preconditions and effects)
    task_planning/planning.py   the problem, UPF + Fast Downward, the expected states, executing a plan
    state_estimation.py         pixels -> world points, where each item rests, which mask is which
                                item, a grasp proposed in the image and its 3D pose
    motion_planning.py          inverse kinematics choice, the held item's box, the motions of the
                                camera views and of pick-up / put-on-table / put-in-basket with
                                MoveIt, where to put an item down, the obstacles MoveIt avoids

What these files may use from the rest of the code is in lab_replica/toolkit.py
(data types, geometric helpers, the robot's kinematics, the steps of a plan)
and lab_replica/moveit_client.py (MoveIt, without ROS in its interface).

Who calls what:
    scripts/goal_console.py, tamp/task.py     -> task_planning/planning.py
    perception/rgbd.py, perception/scene.py   -> state_estimation.py (points, placements, masks)
    autonomy/executive.py                     -> state_estimation.py (grasps), motion_planning.py
"""
