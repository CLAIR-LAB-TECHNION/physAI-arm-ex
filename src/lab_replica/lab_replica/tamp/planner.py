"""Task planning with the Unified Planning Framework (UPF) and Fast Downward.

The domain (config/domain.pddl) is read once with UPF's PDDL reader: its
type `item`, its fluents (the predicates) and its actions. A planning problem
is then built in UPF from it: the twelve items as objects, the initial state
(a set of facts such as ("on-table", "red-can")) and the goal formula
(goal.py: and, or, not, imply, forall, exists). UPF's Fast Downward engine
(`up-fast-downward`, README section T.1) solves it:

    with OneshotPlanner(name="fast-downward") as planner:
        result = planner.solve(problem)

Its default search is LAMA's first iteration (satisficing: the plan is not
guaranteed to be the shortest). UPF compiles disjunctions and quantifiers in
the goal away before Fast Downward sees the problem.
"""

import functools
import os

from .. import config
from ..planning import pddl

ENGINE = "fast-downward"


class PlannerError(RuntimeError):
    pass


class Unsolvable(PlannerError):
    pass


def check():
    """Raise PlannerError when UPF or its Fast Downward engine is missing."""
    try:
        from unified_planning.shortcuts import get_environment
    except ImportError as e:
        raise PlannerError(f"the Unified Planning Framework is not installed ({e}): README section T.1")
    if ENGINE not in get_environment().factory.engines:
        raise PlannerError(f"UPF has no {ENGINE!r} engine: pip install up-fast-downward (README section T.1)")


@functools.lru_cache(maxsize=1)
def domain():
    """The domain as a UPF problem without objects, initial state or goal."""
    from unified_planning.io import PDDLReader
    from unified_planning.shortcuts import get_environment

    get_environment().credits_stream = None      # no engine banner on every plan
    return PDDLReader().parse_problem(config.path("config", "domain.pddl"))


def make_problem(init, formula, name="task", items=pddl.ITEMS):
    """A UPF problem: the domain, `items` as objects, the facts of `init`
    true (all others false) and the goal `formula` (goal.py, nested lists)."""
    from unified_planning.shortcuts import Object

    problem = domain().clone()
    problem.name = name
    item = problem.user_type("item")
    objects = {i: Object(i, item) for i in items}
    problem.add_objects(objects.values())
    for f in init:
        problem.set_initial_value(problem.fluent(f[0])(*(objects[a] for a in f[1:])), True)
    problem.add_goal(_expression(problem, formula, objects, {}))
    return problem


def _expression(problem, e, objects, variables):
    """A goal formula (nested lists of strings) as a UPF expression."""
    from unified_planning.shortcuts import And, Exists, Forall, Implies, Not, Or, Variable

    head = e[0]
    if head == "and":
        return And(*(_expression(problem, x, objects, variables) for x in e[1:]))
    if head == "or":
        return Or(*(_expression(problem, x, objects, variables) for x in e[1:]))
    if head == "not":
        return Not(_expression(problem, e[1], objects, variables))
    if head == "imply":
        return Implies(_expression(problem, e[1], objects, variables),
                       _expression(problem, e[2], objects, variables))
    if head in ("forall", "exists"):
        item = problem.user_type("item")
        names = [v for v in e[1] if v.startswith("?")]          # (?x ?y - item)
        vs = [Variable(v[1:].replace("-", "_"), item) for v in names]
        inner = _expression(problem, e[2], objects, {**variables, **dict(zip(names, vs))})
        return (Forall if head == "forall" else Exists)(inner, *vs)
    args = [variables[a] if a.startswith("?") else objects[a] for a in e[1:]]
    return problem.fluent(head)(*args)


def action_text(a):
    """A UPF action instance as PDDL text: '(pick-up green-mug)'."""
    return "(" + " ".join([a.action.name] + [str(p) for p in a.actual_parameters]) + ")"


def plan(problem, out_dir=None, time_limit=60):
    """Solve a UPF problem (make_problem) with OneshotPlanner(name="fast-downward").
    Returns the list of actions, e.g. ['(pick-up green-mug)', '(put-in-basket green-mug)'].

    With out_dir, the plan (plan.txt) and the planner's output (planner.log)
    are written there. Raises Unsolvable when the goal cannot be reached,
    PlannerError on any other failure."""
    from unified_planning.engines import PlanGenerationResultStatus as Status
    from unified_planning.shortcuts import OneshotPlanner

    check()
    with OneshotPlanner(name=ENGINE) as planner:
        result = planner.solve(problem, timeout=time_limit)
    output = "\n".join(str(m.message) for m in result.log_messages or [])
    if out_dir:
        with open(os.path.join(out_dir, "planner.log"), "a") as f:
            f.write(f"UPF OneshotPlanner(name={ENGINE!r}) ({result.engine_name}): {result.status.name}\n"
                    f"{output}\n\n")
    if result.status in (Status.UNSOLVABLE_PROVEN, Status.UNSOLVABLE_INCOMPLETELY):
        raise Unsolvable(f"{ENGINE}: the goal cannot be reached from the initial state")
    if result.status not in (Status.SOLVED_SATISFICING, Status.SOLVED_OPTIMALLY) or result.plan is None:
        tail = "\n".join(output.strip().splitlines()[-8:])
        raise PlannerError(f"{ENGINE} failed ({result.status.name})" + (f":\n{tail}" if tail else ""))
    actions = [action_text(a) for a in result.plan.actions]
    if out_dir:
        with open(os.path.join(out_dir, "plan.txt"), "w") as f:
            f.write("".join(a + "\n" for a in actions) + f"; cost = {len(actions)} (unit cost)\n")
    return actions
