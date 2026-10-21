"""Task planning with the Unified Planning Framework (UPF) and Fast Downward.

The goal console (scripts/goal_console.py) reads problem.pddl, puts in the
current state and the goal typed (tamp/problem_file.py), then uses these
functions in turn:

    problem  = load_problem(text)                  domain.pddl + the problem text, read by UPF
    plan     = solve(problem)                      the plan (UPF action instances)
    states   = expected_states(problem, plan)      the state after each action
    execute_plan(state, actions, states, perform)  the robot performs them

States are sets of facts, each a tuple of strings:
    ("on-table", "red-can"), ("in-basket", "green-mug"),
    ("robot-gripping", "red-can"), ("robot-gripper-empty",)

An action is a string such as "(pick-up red-can)" (planner.action_text turns
a UPF action instance into one).
"""

import os

from ...tamp.planner import PlannerError, Unsolvable

HERE = os.path.dirname(os.path.realpath(__file__))
DOMAIN_FILE = os.path.join(HERE, "domain.pddl")
PROBLEM_FILE = os.path.join(HERE, "problem.pddl")


def load_problem(problem_text, domain_file=DOMAIN_FILE):
    """The UPF problem of domain_file and this problem text (UPF's PDDLReader)."""
    from unified_planning.io import PDDLReader

    with open(domain_file) as f:
        domain_text = f.read()
    return PDDLReader().parse_problem_string(domain_text, problem_text)


def solve(problem, time_limit=60):
    """A plan for the UPF problem from Fast Downward: the list of its action
    instances (unified_planning.plans.ActionInstance).

    Raises Unsolvable when the goal cannot be reached, PlannerError when the
    planner fails otherwise (e.g. it runs out of time)."""
    from unified_planning.engines import PlanGenerationResultStatus as Status
    from unified_planning.shortcuts import OneshotPlanner

    with OneshotPlanner(name="fast-downward") as planner:
        result = planner.solve(problem, timeout=time_limit)
    if result.status in (Status.UNSOLVABLE_PROVEN, Status.UNSOLVABLE_INCOMPLETELY):
        raise Unsolvable("the goal cannot be reached from the initial state")
    if result.status not in (Status.SOLVED_SATISFICING, Status.SOLVED_OPTIMALLY) or result.plan is None:
        raise PlannerError(f"Fast Downward failed ({result.status.name})")
    return list(result.plan.actions)


def facts_of(problem, upf_state):
    """The facts (see the module docstring) that hold in a state of UPF's
    SequentialSimulator."""
    import itertools

    facts = set()
    for fluent in problem.fluents:
        # Every grounding: each parameter takes every object of its type.
        groundings = itertools.product(*(problem.objects(p.type) for p in fluent.signature))
        for args in groundings:
            if upf_state.get_value(fluent(*args)).is_true():
                facts.add((fluent.name,) + tuple(str(o) for o in args))
    return frozenset(facts)


def expected_states(problem, plan):
    """The state after each action of `plan` (a list of UPF action
    instances), simulated from the problem's initial state with UPF's
    SequentialSimulator: a list of fact sets, one per action."""
    from unified_planning.shortcuts import SequentialSimulator

    states = []
    with SequentialSimulator(problem=problem) as simulator:
        s = simulator.get_initial_state()
        for a in plan:
            s = simulator.apply(s, a.action, a.actual_parameters)
            if s is None:
                raise PlannerError(f"{a} is not applicable in the simulated state")
            states.append(facts_of(problem, s))
    return states


def execute_plan(state, actions, states, perform, before=None, after=None):
    """Perform the plan's actions one after the other, stopping at the first
    one that fails.

    state    the initial state (a fact set)
    actions  the plan's actions as strings, e.g. ["(pick-up red-can)", ...]
    states   expected_states of the plan: the state after each action
    perform  perform(action) -> (success, message): the robot does it
    before   before(k, action) -> bool, called before action k (0-based);
             False stops the plan there (None: always go on)
    after    after(k, action, success, message), called after action k

    Returns (state, failed_at): the state the performed actions should have
    produced, and the index of the action that failed or was not performed
    (None when the whole plan was performed)."""
    for k, action in enumerate(actions):
        if before is not None and not before(k, action):
            return state, k
        ok, message = perform(action)
        if after is not None:
            after(k, action, ok, message)
        if not ok:
            return state, k
        state = states[k]
    return state, None
