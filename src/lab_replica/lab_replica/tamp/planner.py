"""Task planning: what the framework adds around the exercise's planner.

The planning itself is in exercise/task_planning/ (the domain, the problem
text, UPF's PDDLReader, OneshotPlanner(name="fast-downward") and
SequentialSimulator). This module has the errors it raises, the check that
UPF and its Fast Downward engine are installed (README section T.1), and
the conversion of UPF action instances to text.
"""

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
    env = get_environment()
    env.credits_stream = None      # no engine banner on every plan
    if ENGINE not in env.factory.engines:
        raise PlannerError(f"UPF has no {ENGINE!r} engine: pip install up-fast-downward (README section T.1)")


def action_text(a):
    """A UPF action instance as PDDL text: '(pick-up green-mug)'."""
    return "(" + " ".join([a.action.name] + [str(p) for p in a.actual_parameters]) + ")"
