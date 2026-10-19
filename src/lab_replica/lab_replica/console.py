"""Interactive consoles: type a PDDL action, or a goal, and the robot does it.

Both run the autonomous skills (autonomy/executive.py) in this process: one
ROS node holds the arm and gripper clients and the wrist camera
subscriptions, and SAM 3 is loaded once at start. The prompt appears only
when the arm, the gripper and the camera answer and SAM 3 is loaded.

    scripts/action_console.py   one grounded action at a time
    scripts/goal_console.py     a goal: Fast Downward plans, the actions run in turn

Neither knows whether it drives the simulation or the real robot, except
for the clock (--sim: simulation time).
"""

import difflib
import os
import sys
import time

from .planning import pddl

ACTIONS = ("pick-up", "put-in-basket", "put-on-table")
PREDICATES = ("on-table", "in-basket", "robot-gripping", "robot-gripper-empty")


class InvalidAction(ValueError):
    pass


def parse_action(text):
    """'(pick-up green-mug)' (parentheses and case optional) -> '(pick-up green-mug)'.
    Raises InvalidAction with the reason."""
    words = text.strip().lower().replace("(", " ").replace(")", " ").split()
    if len(words) != 2:
        raise InvalidAction("an action is a name and one item, e.g. (pick-up green-mug)")
    name, item = words
    if name not in ACTIONS:
        close = difflib.get_close_matches(name, ACTIONS, n=1, cutoff=0.5)
        raise InvalidAction(f"unknown action {name!r}" + (f" (did you mean {close[0]}?)" if close else ""))
    if item not in pddl.ITEMS:
        close = difflib.get_close_matches(item, pddl.ITEMS, n=1, cutoff=0.4)
        raise InvalidAction(f"unknown item {item!r}" + (f" (did you mean {close[0]}?)" if close else ""))
    return f"({name} {item})"


# ---- terminal ------------------------------------------------------------------------------

class Terminal:
    """Prompt with history and Tab completion; a status line that is
    rewritten in place while an action runs."""

    def __init__(self, words, history_name):
        self.tty = sys.stdout.isatty()
        self.status_shown = False
        try:
            import readline
        except ImportError:
            return
        self.words = sorted(set(words))
        readline.set_completer_delims(" \t\n()")
        readline.set_completer(self._complete)
        readline.parse_and_bind("tab: complete")
        self.history = os.path.join(os.path.expanduser("~"), ".ros", f"lab_{history_name}_history")
        try:
            readline.read_history_file(self.history)
        except OSError:
            pass
        readline.set_history_length(500)
        import atexit
        atexit.register(self._save_history, readline)

    def _save_history(self, readline):
        try:
            os.makedirs(os.path.dirname(self.history), exist_ok=True)
            readline.write_history_file(self.history)
        except OSError:
            pass

    def _complete(self, text, state):
        hits = [w for w in self.words if w.startswith(text.lower())]
        return hits[state] if state < len(hits) else None

    def bold(self, s):
        return f"\033[1m{s}\033[0m" if self.tty else s

    def dim(self, s):
        return f"\033[2m{s}\033[0m" if self.tty else s

    def colour(self, s, ok):
        return f"\033[1;{32 if ok else 31}m{s}\033[0m" if self.tty else s

    def status(self, text):
        """The current phase of the running action."""
        if self.tty:
            width = max(20, os.get_terminal_size().columns - 1) if sys.stdout.isatty() else 100
            sys.stdout.write("\r\033[K" + self.dim(f"  ... {text}"[:width]))
            sys.stdout.flush()
            self.status_shown = True
        else:
            print(f"  ... {text}", flush=True)

    def end_status(self):
        if self.status_shown:
            sys.stdout.write("\r\033[K")
            sys.stdout.flush()
            self.status_shown = False

    def say(self, text=""):
        self.end_status()
        print(text, flush=True)

    def ask(self, prompt):
        """A line from the user; None on Ctrl-D."""
        self.end_status()
        try:
            return input(prompt)
        except EOFError:
            print()
            return None


def wrap_words(words, indent, width=100):
    """Words joined into lines of at most `width` characters, each but the
    first indented by `indent` spaces."""
    lines, cur = [], ""
    for w in words:
        if cur and len(cur) + 2 + len(w) > width - indent:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur}  {w}" if cur else w
    lines.append(cur)
    return ("\n" + " " * indent).join(lines)


# ---- the robot -------------------------------------------------------------------------------

class Robot:
    """The autonomous skills in this process: arm, gripper, wrist camera, SAM 3.

    log_file receives the executive's detailed log (the terminal shows the
    phases only, unless verbose)."""

    def __init__(self, term, sim=False, speed_scale=1.0, run_dir=None, verbose=False, return_home=True,
                 save_frames=False):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions

        from . import config
        from .execution.executor import Executor, active_trajectory_controller

        self.term = term
        self.rclpy = rclpy
        self.run_dir = run_dir or os.path.join(config.workspace_root(), "runs", time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.run_dir, "console.log"), "a", buffering=1)
        self.verbose = verbose
        # Ctrl-C must reach this program (to stop the arm), not shut ROS down.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

        step = self._step
        step("connecting to the arm and the gripper")
        probe = rclpy.create_node("lab_console_probe")
        if sim:
            probe.set_parameters([Parameter("use_sim_time", value=True)])
        try:
            controller = active_trajectory_controller(probe, timeout=30.0)
        except Exception as e:  # noqa: BLE001
            raise SystemExit(f"\nno arm: {e}\n  Is the robot running? (run/1_robot.sh {'sim' if sim else 'real'})")
        finally:
            probe.destroy_node()
        self.node = Executor(controller=controller, speed_scale=speed_scale)
        if sim:
            self.node.set_parameters([Parameter("use_sim_time", value=True)])
        try:
            self.node.wait_ready(timeout=30.0)
        except Exception as e:  # noqa: BLE001
            raise SystemExit(f"\nno arm or gripper: {e}\n  Is the robot running? "
                             f"(run/1_robot.sh {'sim' if sim else 'real'})")
        self._done(f"controller {controller}")

        step("loading SAM 3 (the segmentation model)")
        t0 = time.time()
        os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
        os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")
        import warnings
        warnings.filterwarnings("ignore", category=FutureWarning)
        from .autonomy.executive import ActionExecutive
        self.ex = ActionExecutive(self.node, log=self._log, run_dir=self.run_dir, return_home=return_home,
                                  save_frames=save_frames)
        self._done(f"{time.time() - t0:.0f} s")

        step("checking the gripper")
        try:
            self.holding = self.ex.holding()
        except Exception as e:  # noqa: BLE001
            raise SystemExit(f"\n{e}\n  The gripper driver starts with run/1_robot.sh.")
        self._done("holding something" if self.holding else "empty")

        step("waiting for the wrist camera")
        try:
            f = self.ex.cam.capture(after=self.ex.now() - 1e6, timeout=20.0)
        except TimeoutError:
            raise SystemExit(f"\nno camera images on {self.ex.cfg['topics']['color']} and "
                             f"{self.ex.cfg['topics']['depth']}\n  Is the camera running? "
                             f"(run/2_camera.sh {'sim' if sim else 'real'})")
        self._done(f"{f.rgb.shape[1]} x {f.rgb.shape[0]}")
        self.ex.on_phase = self._phase

    def _step(self, text):
        sys.stdout.write(f"  {text} ... ")
        sys.stdout.flush()

    def _done(self, text):
        print(f"ok ({text})" if text else "ok", flush=True)

    def _log(self, text):
        self.log_file.write(f"{time.strftime('%H:%M:%S')} {text}\n")
        if self.verbose:
            self.term.say(self.term.dim(text))

    def _phase(self, text):
        if text.startswith(("done:", "failed:")):
            return
        if not self.verbose:
            self.term.status(text)

    def grip(self):
        """The gripper's grip-detected flag, or None without a reading."""
        try:
            return self.ex.holding()
        except Exception:  # noqa: BLE001
            return None

    def execute(self, action):
        """Run one action. Returns (success, message). Ctrl-C stops the arm
        and fails the action."""
        try:
            ok, msg = self.ex.execute(action)
        except KeyboardInterrupt:
            n = self.node.cancel_active()
            ok, msg = False, "interrupted (Ctrl-C)" + ("; the arm was stopped" if n else "")
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f"{type(e).__name__}: {e}"
        self._log(f"   {'OK' if ok else 'FAILED'}: {msg}")
        self.term.end_status()
        return ok, msg

    def shutdown(self):
        try:
            self.node.cancel_active()
        except Exception:  # noqa: BLE001
            pass
        self.rclpy.try_shutdown()
        self.log_file.close()


def common_arguments(ap):
    g = ap.add_argument_group("robot")
    g.add_argument("--sim", action="store_true", help="the simulation (run/1_robot.sh sim): use its clock")
    g.add_argument("--speed-scale", type=float, default=1.0,
                   help="scale all motion speeds (e.g. 0.3 for a first run on the real robot)")
    g.add_argument("--no-home", action="store_true", help="do not return to the home pose after each action")
    g.add_argument("--verbose", action="store_true", help="print every step of the skills, not only the phases")
    g.add_argument("--save-frames", action="store_true",
                   help="also save every camera frame (colour, depth, arm joints) in the run directory")
    return ap


def banner(term, title, sim):
    term.say(term.bold(f"lab-tamp: {title}") + f"  ({'simulation' if sim else 'REAL ROBOT'})")
