"""Executing a task plan with the autonomous skills.

Every action goes to the skill server (scripts/skill_server.py) through
/lab_skills/execute, one after the other. The skill server looks at the
scene, plans and executes each action by itself; this module only orders the
actions, keeps track of the PDDL state they should produce, and stops at the
first action that fails (a refused action leaves the scene as it was).

When the initial state has the robot holding an item, that item stands on
the table (the simulated world cannot start with it in the gripper), so a
setup action `(pick-up X)` comes first, unless the gripper already holds
something.
"""

import time

from ..planning import pddl


class SkillClient:
    """Action client of /lab_skills/execute on `node` (spun from here)."""

    def __init__(self, node, log=print):
        import rclpy
        from rclpy.action import ActionClient
        from std_msgs.msg import Bool

        from lab_skills_msgs.action import ExecuteAction
        self.rclpy = rclpy
        self.node = node
        self.log = log
        self.Goal = ExecuteAction.Goal
        self.client = ActionClient(node, ExecuteAction, "/lab_skills/execute")
        self.grip = None
        node.create_subscription(Bool, "/rg6/grip_detected", lambda m: setattr(self, "grip", m.data), 10)

    def wait(self, timeout=30.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def gripper_holds(self, timeout=5.0):
        """The RG6's grip-detected flag (None when nothing is published)."""
        self.grip = None
        t0 = time.time()
        while self.grip is None and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.grip

    def execute(self, action, timeout=900.0):
        """Run one action. Returns dict(ok, message, held, seconds)."""
        t0 = time.time()
        last = [""]

        def feedback(f):
            if f.feedback.phase != last[0]:
                last[0] = f.feedback.phase
                self.log(f"     {f.feedback.phase}")
        fut = self.client.send_goal_async(self.Goal(action=action), feedback_callback=feedback)
        while not fut.done():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        gh = fut.result()
        if not gh.accepted:
            return dict(ok=False, message="rejected by the skill server", held="", seconds=0.0)
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if not rf.done():
            return dict(ok=False, message=f"no result after {timeout:.0f} s", held="", seconds=time.time() - t0)
        r = rf.result().result
        return dict(ok=bool(r.success), message=r.message, held=r.held, seconds=round(time.time() - t0, 1))


def effect(state, action):
    """The PDDL state after `action`, or None if it is not applicable. The
    item may be any (also one added with new-item:)."""
    item = action.strip().strip("()").split()[-1]
    for a, s in pddl.successors(state, items=[item]):
        if a == action:
            return s
    return None


def execute_plan(skills, init, plan, log=print, setup=True, step=False, on_action=None):
    """Execute `plan` from `init`. `on_action(k, action, status)`, if given,
    is called with status "running", then "ok" or "failed" (k = -1 for the
    setup action). Returns a report dict:
        ok           every action succeeded
        actions      [{action, ok, message, held, seconds, setup?}]
        state        the PDDL state the executed actions should have produced
        failed_at    index of the failed plan action (None)"""
    state = frozenset(init)
    rows = []
    report = dict(ok=False, actions=rows, failed_at=None)
    held = pddl.held(state)
    note = on_action or (lambda k, a, st: None)
    if held is not None and setup:
        grip = skills.gripper_holds()
        if grip:
            log(f"== setup: the gripper already holds something; taken to be the {held}")
        else:
            log(f"== setup: (pick-up {held}) (the initial state has the robot holding it)")
            note(-1, f"(pick-up {held})", "running")
            r = skills.execute(f"(pick-up {held})")
            note(-1, f"(pick-up {held})", "ok" if r["ok"] else "failed")
            rows.append(dict(action=f"(pick-up {held})", setup=True, **r))
            log(f"   {'OK' if r['ok'] else 'FAILED'}: {r['message']} ({r['seconds']:.0f} s)")
            if not r["ok"]:
                report["state"] = sorted(map(pddl.fact_str, state))
                return report
    for k, action in enumerate(plan):
        nxt = effect(state, action)
        if nxt is None:
            raise ValueError(f"{action} is not applicable in the planned state: the plan does not fit the init")
        if step:
            input(f"   Enter to run action {k + 1}/{len(plan)} {action} (Ctrl-C to stop) ")
        log(f"== {k + 1}/{len(plan)} {action}")
        note(k, action, "running")
        r = skills.execute(action)
        note(k, action, "ok" if r["ok"] else "failed")
        rows.append(dict(action=action, **r))
        log(f"   {'OK' if r['ok'] else 'FAILED'}: {r['message']} ({r['seconds']:.0f} s)")
        if not r["ok"]:
            report["failed_at"] = k
            break
        expect_held = pddl.held(nxt) or ""
        if r["held"] != expect_held:
            log(f"   warning: the skill server reports holding {r['held'] or 'nothing'}, "
                f"expected {expect_held or 'nothing'}")
        state = nxt
    else:
        report["ok"] = True
    report["state"] = sorted(map(pddl.fact_str, state))
    return report
