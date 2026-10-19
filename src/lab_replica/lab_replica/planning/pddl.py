"""The order-assembly PDDL domain: problem files, states and an optimal planner.

The domain file (config/domain.pddl) is the one from the lab data set, used
unchanged. States are frozensets of ground facts such as
("on-table", "red-can") or ("robot-gripper-empty",).
"""

import re
from collections import deque

from .. import config

ITEMS = [
    "red-can", "red-spray-bottle", "roll-of-black-tape", "roll-of-green-tape", "roll-of-blue-tape",
    "blue-cup-with-yellow-stripe", "green-mug", "yellow-block", "orange-block", "blue-block",
    "red-block", "green-block",
]
EMPTY = ("robot-gripper-empty",)


def domain_text():
    with open(config.path("config", "domain.pddl")) as f:
        return f.read()


def fact_str(f):
    return "(" + " ".join(f) + ")"


def parse_facts(text):
    return [tuple(m.split()) for m in re.findall(r"\(([a-z-]+(?: [a-z-]+)?)\)", text)]


def held(state):
    for f in state:
        if f[0] == "robot-gripping":
            return f[1]
    return None


def successors(state, items=ITEMS):
    """(action string, next state) for every applicable action."""
    out = []
    h = held(state)
    if EMPTY in state:
        for i in items:
            if ("on-table", i) in state:
                s = set(state)
                s -= {("on-table", i), EMPTY}
                s.add(("robot-gripping", i))
                out.append((f"(pick-up {i})", frozenset(s)))
    if h is not None:
        for pred in ("on-table", "in-basket"):
            s = set(state)
            s.discard(("robot-gripping", h))
            s |= {(pred, h), EMPTY}
            name = "put-on-table" if pred == "on-table" else "put-in-basket"
            out.append((f"({name} {h})", frozenset(s)))
    return out


def satisfies(state, goal):
    return all(g in state for g in goal)


def plan_bfs(init, goal, items=ITEMS, max_len=30):
    """Shortest plan (list of action strings) or None."""
    init = frozenset(init)
    if satisfies(init, goal):
        return []
    prev = {init: None}
    q = deque([init])
    while q:
        s = q.popleft()
        for act, n in successors(s, items):
            if n in prev:
                continue
            prev[n] = (s, act)
            if satisfies(n, goal):
                plan = []
                while prev[n] is not None:
                    n, a = prev[n]
                    plan.append(a)
                return plan[::-1]
            q.append(n)
    return None


def problem_text(init, goal, items=ITEMS, name="order-assembly-problem", goal_formula=None):
    """Problem file in the same layout as the lab data set. `goal_formula`
    (PDDL text) replaces the conjunction of the `goal` facts."""
    objs = "\n".join(f"      {i}" for i in items)
    order = {i: k for k, i in enumerate(items)}

    def key(f):
        return (order.get(f[1], 99) if len(f) > 1 else 100, f[0])

    init_s = "\n".join(f"        {fact_str(f)}" for f in sorted(init, key=key))
    goal_s = goal_formula or "(and " + " ".join(fact_str(f) for f in sorted(goal, key=key)) + ")"
    return (f"(define (problem {name}) \n    (:domain order-assembly-domain)\n    (:objects\n{objs}\n      - item\n    )\n"
            f"    (:init\n{init_s}\n    )\n    (:goal {goal_s})\n)\n")


def read_problem(path):
    text = open(path).read()
    init_part = text[text.index("(:init"):text.index("(:goal")]
    goal_part = text[text.index("(:goal"):]
    return frozenset(parse_facts(init_part)), frozenset(parse_facts(goal_part))
