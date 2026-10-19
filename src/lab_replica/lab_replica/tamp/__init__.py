"""Task and motion planning: a goal over the fixed domain and objects,
planned with Fast Downward and executed with the autonomous skills.

    goal.py           parsing, checking and evaluating goal formulas
    fast_downward.py  running the planner
    task.py           a task = scene (initial state) + goal; its directory
    execute.py        executing a plan through the skill server
    sim_truth.py      the simulator's true PDDL state, for checking the goal
"""
