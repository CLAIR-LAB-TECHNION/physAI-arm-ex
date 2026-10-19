"""Task planning with Fast Downward.

Fast Downward is found at $FAST_DOWNWARD (the fast-downward.py driver), else
in <workspace>/third_party/downward (README section T.1), else as
`fast-downward` on the PATH.

The search is optimal A* with LM-cut. LM-cut does not support the axioms that
the translator makes from disjunctions and quantifiers in the goal; Fast
Downward then exits with "unsupported", and the search is repeated with
blind A* (still optimal; the state space of this domain has a few ten
thousand states).
"""

import os
import shutil
import subprocess
import sys
import tempfile

from .. import config

SEARCH = "astar(lmcut())"
FALLBACK_SEARCH = "astar(blind())"

# driver/returncodes.py
UNSOLVABLE = {10, 11, 12}
UNSUPPORTED = 34


class PlannerError(RuntimeError):
    pass


class Unsolvable(PlannerError):
    pass


def driver():
    """Command (list) that runs the fast-downward.py driver."""
    env = os.environ.get("FAST_DOWNWARD")
    if env:
        return [sys.executable, env] if env.endswith(".py") else [env]
    local = os.path.join(config.workspace_root(), "third_party", "downward", "fast-downward.py")
    if os.path.exists(local):
        return [sys.executable, local]
    exe = shutil.which("fast-downward") or shutil.which("fast-downward.py")
    if exe:
        return [exe]
    raise PlannerError("Fast Downward not found: build it in third_party/downward (README section T.1) "
                       "or set FAST_DOWNWARD to its fast-downward.py")


def plan(domain_file, problem_file, out_dir=None, time_limit=60, search=SEARCH, log=print):
    """Plan with Fast Downward. Returns the list of actions, e.g.
    ['(pick-up green-mug)', '(put-in-basket green-mug)'].

    With out_dir, the plan (plan.txt, Fast Downward's sas_plan format) and the
    planner's output (planner.log) are written there. Raises Unsolvable when
    the goal cannot be reached, PlannerError on any other failure."""
    work = tempfile.mkdtemp(prefix="lab_fd_")
    try:
        plan_file = os.path.join(work, "sas_plan")
        cmd = driver() + ["--overall-time-limit", f"{time_limit}s", "--plan-file", plan_file,
                          os.path.abspath(domain_file), os.path.abspath(problem_file), "--search", search]
        r = subprocess.run(cmd, cwd=work, capture_output=True, text=True)
        output = r.stdout + r.stderr
        if out_dir:
            with open(os.path.join(out_dir, "planner.log"), "a") as f:
                f.write(f"$ {' '.join(cmd)}\n{output}\nexit code {r.returncode}\n\n")
        if r.returncode == UNSUPPORTED and search != FALLBACK_SEARCH:
            log(f"  {search} does not support this goal (disjunction or quantifier): using {FALLBACK_SEARCH}")
            return plan(domain_file, problem_file, out_dir, time_limit, FALLBACK_SEARCH, log)
        if r.returncode in UNSOLVABLE:
            raise Unsolvable("Fast Downward: the goal cannot be reached from the initial state")
        if r.returncode != 0 or not os.path.exists(plan_file):
            tail = "\n".join(output.strip().splitlines()[-8:])
            raise PlannerError(f"Fast Downward failed (exit code {r.returncode}):\n{tail}")
        with open(plan_file) as f:
            text = f.read()
        if out_dir:
            with open(os.path.join(out_dir, "plan.txt"), "w") as f:
                f.write(text)
        return [line.strip() for line in text.splitlines() if line.strip() and not line.startswith(";")]
    finally:
        shutil.rmtree(work, ignore_errors=True)
