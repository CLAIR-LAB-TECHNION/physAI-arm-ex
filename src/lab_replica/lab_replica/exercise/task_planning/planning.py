"""Task planning with the Unified Planning Framework (UPF) and Fast Downward.

The goal console (scripts/goal_console.py) reads problem.pddl, puts in the
current state and the goal typed (tamp/problem_file.py), then uses these
functions in turn:

    problem  = load_problem(text)                  domain.pddl + the problem text, read by UPF
    plan     = solve(problem)                      the plan (UPF action instances)
    states   = expected_states(problem, plan)      the state after each action
    execute_plan(state, actions, states, perform)  the robot performs them

States are sets of facts, each a tuple of strings:
    ("on-table", "red-block"), ("in-basket", "blue-block"),
    ("robot-gripping", "red-block"), ("robot-gripper-empty",)

An action is a string such as "(pick-up red-block)".

UPF documentation: https://unified-planning.readthedocs.io/en/latest/
    reading PDDL       https://unified-planning.readthedocs.io/en/latest/notebooks/io/01-pddl-usage-example.html
    solving a problem  https://unified-planning.readthedocs.io/en/latest/notebooks/01-basic-example.html
    simulating a plan  https://unified-planning.readthedocs.io/en/latest/notebooks/08-sequential-simulator.html
Fast Downward (the planner UPF calls): https://www.fast-downward.org/
"""

import os

from ...tamp.planner import PlannerError, Unsolvable  # noqa: F401  (raised by solve)

HERE = os.path.dirname(os.path.realpath(__file__))
DOMAIN_FILE = os.path.join(HERE, "domain.pddl")
PROBLEM_FILE = os.path.join(HERE, "problem.pddl")


def load_problem(problem_text, domain_file=DOMAIN_FILE):
    """The UPF problem (unified_planning.model.Problem) of the PDDL domain in
    `domain_file` and the PDDL problem `problem_text` (a string, not a file).

    Useful: unified_planning.io.PDDLReader can read a domain and a problem
    given as strings."""
    # TODO
    raise NotImplementedError("planning.load_problem (exercise/task_planning/planning.py)")


def solve(problem, time_limit=60):
    """A plan for the UPF problem from Fast Downward: the list of its action
    instances (unified_planning.plans.ActionInstance), in order.

    Raises Unsolvable when the goal cannot be reached, PlannerError when the
    planner fails otherwise (e.g. it runs out of time). Both come from
    tamp/planner.py and are imported above.

    Useful: unified_planning.shortcuts.OneshotPlanner(name="fast-downward")
    (use it in a `with` block). Its solve() result has a .status (a
    unified_planning.engines.PlanGenerationResultStatus, which tells a
    proven unsolvable problem from a solved one) and a .plan."""
    # TODO
    raise NotImplementedError("planning.solve (exercise/task_planning/planning.py)")


def facts_of(problem, upf_state):
    """The facts (see the module docstring) that hold in a state of UPF's
    SequentialSimulator, as a frozenset.

    Hints: a fact is a fluent (problem.fluents) applied to objects of the
    types of its parameters (fluent.signature, problem.objects(type)). A
    fluent without parameters is a fact too. Ask the state for the value of
    each grounded fluent (upf_state.get_value(...).is_true())."""
    # TODO
    raise NotImplementedError("planning.facts_of (exercise/task_planning/planning.py)")


def expected_states(problem, plan):
    """The state after each action of `plan` (a list of UPF action
    instances), simulated from the problem's initial state: a list of fact
    sets (facts_of), one per action.

    Raises PlannerError when an action is not applicable where the plan
    performs it.

    Useful: unified_planning.shortcuts.SequentialSimulator(problem=problem)
    (in a `with` block): get_initial_state(), apply(state, action, parameters).
    An action instance has .action and .actual_parameters."""
    # TODO
    raise NotImplementedError("planning.expected_states (exercise/task_planning/planning.py)")


def execute_plan(state, actions, states, perform, before=None, after=None):
    """Perform the plan's actions one after the other, stopping at the first
    one that fails.

    state    the initial state (a fact set)
    actions  the plan's actions as strings, e.g. ["(pick-up red-block)", ...]
    states   expected_states of the plan: the state after each action
    perform  perform(action) -> (success, message): the robot does it
    before   before(k, action) -> bool, called before action k (0-based);
             False stops the plan there (None: always go on)
    after    after(k, action, success, message), called after action k

    Returns (state, failed_at): the state the performed actions should have
    produced, and the index of the action that failed or was not performed
    (None when the whole plan was performed)."""
    # TODO
    raise NotImplementedError("planning.execute_plan (exercise/task_planning/planning.py)")
