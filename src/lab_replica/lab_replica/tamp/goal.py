"""The goal of a task: a PDDL goal formula over the fixed domain and objects.

A goal is given as text, in any of these forms:

    (in-basket green-mug)                              one fact
    (in-basket green-mug) (robot-gripping red-can)     several facts (a conjunction)
    (and (in-basket green-mug) (not (on-table red-can)))
    a .pddl problem file                               its (:goal ...) is used

The formula may use and, or, not, imply, forall and exists (quantified
variables are of type item). It is checked against the domain (predicate
names and arities) and the items (the twelve, and those a console added
with new-item:), so a typo fails here with a suggestion instead of in the
planner.
"""

import difflib
import os
import re

from ..planning import pddl

PREDICATES = {"on-table": 1, "robot-gripping": 1, "in-basket": 1, "robot-gripper-empty": 0}
CONNECTIVES = ("and", "or", "not", "imply", "forall", "exists")


class GoalError(ValueError):
    pass


# ---- s-expressions -------------------------------------------------------------------

def _tokens(text):
    text = re.sub(r";[^\n]*", " ", text)            # PDDL comments
    return re.findall(r"\(|\)|[^\s()]+", text.lower())


def parse_sexprs(text):
    """All top-level s-expressions of `text` as nested lists of strings."""
    toks = _tokens(text)
    out, stack = [], []
    for t in toks:
        if t == "(":
            stack.append([])
        elif t == ")":
            if not stack:
                raise GoalError("unbalanced ')'")
            e = stack.pop()
            (stack[-1] if stack else out).append(e)
        else:
            if not stack:
                raise GoalError(f"{t!r} outside parentheses")
            stack[-1].append(t)
    if stack:
        raise GoalError("unbalanced '(': missing ')'")
    return out


def to_text(e):
    return "(" + " ".join(to_text(x) if isinstance(x, list) else x for x in e) + ")"


# ---- reading a goal --------------------------------------------------------------------

def goal_from_problem_file(path):
    """The (:goal ...) formula of a PDDL problem file, and its objects (or None)."""
    exprs = parse_sexprs(open(path).read())
    if not exprs or exprs[0][:1] != ["define"]:
        raise GoalError(f"{path}: not a PDDL problem (no (define ...))")
    goal = objs = None
    for sec in exprs[0][1:]:
        if isinstance(sec, list) and sec[:1] == [":goal"]:
            if len(sec) != 2:
                raise GoalError(f"{path}: (:goal ...) must hold one formula")
            goal = sec[1]
        elif isinstance(sec, list) and sec[:1] == [":objects"]:
            objs = [o for o in sec[1:] if o != "-" and o != "item"]
    if goal is None:
        raise GoalError(f"{path}: no (:goal ...)")
    return goal, objs


def parse_goal(text_or_path, items=pddl.ITEMS):
    """A validated goal formula (nested lists) from a goal text or a problem file.
    `items`: the items it may name (the twelve, and any added in a console).

    Returns (formula, source) where source says where it came from."""
    if os.path.isfile(text_or_path):
        formula, objs = goal_from_problem_file(text_or_path)
        if objs is not None and sorted(objs) != sorted(pddl.ITEMS):
            extra = sorted(set(objs) - set(pddl.ITEMS))
            missing = sorted(set(pddl.ITEMS) - set(objs))
            raise GoalError(f"{text_or_path}: its objects are not the twelve items of the cell"
                            + (f"; unknown: {', '.join(extra)}" if extra else "")
                            + (f"; missing: {', '.join(missing)}" if missing else ""))
        source = f"the :goal of {text_or_path}"
    else:
        exprs = parse_sexprs(text_or_path)
        if not exprs:
            raise GoalError("empty goal")
        formula = exprs[0] if len(exprs) == 1 else ["and", *exprs]
        source = "the command line"
    validate(formula, items=items)
    return formula, source


def validate(e, bound=(), items=pddl.ITEMS):
    """Raise GoalError if `e` is not a formula over the domain and `items`."""
    if not isinstance(e, list) or not e or isinstance(e[0], list):
        raise GoalError(f"not a formula: {to_text(e) if isinstance(e, list) else e}")
    head = e[0]
    if head == "and" or head == "or":
        for x in e[1:]:
            validate(x, bound, items)
    elif head == "not":
        if len(e) != 2:
            raise GoalError(f"(not ...) takes one formula: {to_text(e)}")
        validate(e[1], bound, items)
    elif head == "imply":
        if len(e) != 3:
            raise GoalError(f"(imply A B) takes two formulas: {to_text(e)}")
        validate(e[1], bound, items)
        validate(e[2], bound, items)
    elif head in ("forall", "exists"):
        if len(e) != 3 or not isinstance(e[1], list):
            raise GoalError(f"({head} (?x - item) formula): {to_text(e)}")
        validate(e[2], tuple(bound) + tuple(_variables(e[1])), items)
    elif head in PREDICATES:
        args = e[1:]
        if len(args) != PREDICATES[head]:
            raise GoalError(f"{to_text(e)}: {head} takes {PREDICATES[head]} argument(s)")
        for a in args:
            if isinstance(a, list):
                raise GoalError(f"{to_text(e)}: arguments must be items")
            if a.startswith("?"):
                if a not in bound:
                    raise GoalError(f"{to_text(e)}: {a} is not bound by forall/exists")
            elif a not in items:
                close = difflib.get_close_matches(a, list(items), n=2, cutoff=0.4)
                raise GoalError(f"{to_text(e)}: unknown item {a!r}"
                                + (f" (did you mean {' or '.join(close)}?)" if close else "")
                                + f"; the items are {', '.join(items)}")
    else:
        close = difflib.get_close_matches(head, list(PREDICATES) + list(CONNECTIVES), n=1, cutoff=0.5)
        raise GoalError(f"{to_text(e)}: unknown predicate {head!r}"
                        + (f" (did you mean {close[0]}?)" if close else "")
                        + f"; the predicates are {', '.join(PREDICATES)}")


def _variables(params):
    """['?x', '?y', '-', 'item'] -> ['?x', '?y']."""
    out = []
    for k, p in enumerate(params):
        if p == "-":
            if k + 1 >= len(params) or params[k + 1] != "item":
                raise GoalError(f"variables must be of type item: {to_text(params)}")
        elif p.startswith("?"):
            out.append(p)
        elif p != "item":
            raise GoalError(f"not a variable: {p}")
    return out


# ---- evaluating a goal --------------------------------------------------------------------

def holds(e, facts, env=None, items=pddl.ITEMS):
    """Whether formula `e` holds in the state `facts` (a set of fact tuples);
    forall and exists range over `items`."""
    env = env or {}
    head = e[0]
    if head == "and":
        return all(holds(x, facts, env, items) for x in e[1:])
    if head == "or":
        return any(holds(x, facts, env, items) for x in e[1:])
    if head == "not":
        return not holds(e[1], facts, env, items)
    if head == "imply":
        return not holds(e[1], facts, env, items) or holds(e[2], facts, env, items)
    if head in ("forall", "exists"):
        vs = _variables(e[1])

        def assignments(k, cur):
            if k == len(vs):
                yield cur
                return
            for it in items:
                yield from assignments(k + 1, {**cur, vs[k]: it})
        test = all if head == "forall" else any
        return test(holds(e[2], facts, a, items) for a in assignments(0, env))
    return (head, *(env.get(a, a) for a in e[1:])) in facts


def unsatisfied_facts(e, facts):
    """For a conjunction of facts: the ones that do not hold (else [])."""
    if e[0] in PREDICATES:
        e = ["and", e]
    if e[0] != "and" or any(x[0] not in PREDICATES for x in e[1:]):
        return []
    return [to_text(x) for x in e[1:] if not holds(x, facts)]
