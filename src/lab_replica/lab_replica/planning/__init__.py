"""Planning: from a PDDL plan to collision-checked arm and gripper motions.

    pddl.py        the lab's PDDL domain: problem files, states, optimal planner
    placement.py   where an item rests (support, resting pose, x, y, yaw)
    skills.py      pick-up, put-in-basket, put-on-table, return home as
                   collision-checked motion plans (the parameters of every
                   grasp: opening margin, squeeze, heights, are at its top)
    motion.py      gripper poses, IK along lines and arcs, trajectory timing
    geometry2d.py  2D polygon tests for footprints
"""
