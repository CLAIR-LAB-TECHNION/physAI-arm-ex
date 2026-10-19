#!/usr/bin/env python3
"""Serve the PDDL actions of the cell as a ROS 2 action, executed autonomously.

    ros2 run lab_replica skill_server.py [--sim] [--speed-scale S] [--no-home]

Action /lab_skills/execute (lab_skills_msgs/action/ExecuteAction), e.g.
    ros2 run lab_replica do_action.py "(pick-up green-mug)"
    ros2 action send_goal /lab_skills/execute lab_skills_msgs/action/ExecuteAction \\
        "{action: '(put-in-basket green-mug)'}"

Needs the arm and gripper (sim.launch.py or real.launch.py) and the wrist
RealSense topics (config/perception.yaml). Runs in the workspace's .venv
(torch, transformers for SAM 3). Topics for watching it work:
    /lab_skills/status       std_msgs/String, the current phase
    /lab_skills/detections   sensor_msgs/Image, the last annotated camera view
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

import rclpy  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from lab_replica import config  # noqa: E402
from lab_replica.autonomy.executive import ActionError, ActionExecutive  # noqa: E402
from lab_replica.execution.executor import Executor, active_trajectory_controller  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="use simulation time (with sim.launch.py)")
    ap.add_argument("--speed-scale", type=float, default=1.0, help="scale all motion speeds (0.3 for a first real run)")
    ap.add_argument("--no-home", action="store_true", help="do not return home after each action")
    ap.add_argument("--save-frames", action="store_true", help="also save every camera frame (npz) in the run directory")
    ap.add_argument("--run-dir", default=os.path.join(config.workspace_root(), "runs", time.strftime("%Y%m%d_%H%M%S")),
                    help="where the annotated camera views are saved")
    args, _ = ap.parse_known_args()

    rclpy.init()
    probe = rclpy.create_node("lab_skills_probe")
    controller = active_trajectory_controller(probe)
    probe.destroy_node()
    worker = Executor(controller=controller, speed_scale=args.speed_scale)
    if args.sim:
        worker.set_parameters([Parameter("use_sim_time", value=True)])
    worker.wait_ready()
    log = lambda s: worker.get_logger().info(s)  # noqa: E731
    log("loading SAM 3 ...")
    executive = ActionExecutive(worker, log=log, run_dir=args.run_dir, return_home=not args.no_home,
                                save_frames=args.save_frames)

    from lab_skills_msgs.action import ExecuteAction

    server_node = Node("lab_skills")
    lock = threading.Lock()

    def execute(goal_handle):
        req = goal_handle.request
        feedback = ExecuteAction.Feedback()
        executive.on_phase = lambda text: (setattr(feedback, "phase", text), goal_handle.publish_feedback(feedback))
        res = ExecuteAction.Result()
        with lock:
            try:
                ok, msg = executive.execute(req.action, list(req.target_xy) or None)
            except ActionError as e:
                ok, msg = False, str(e)
            except Exception as e:  # noqa: BLE001
                worker.get_logger().error(f"{type(e).__name__}: {e}")
                ok, msg = False, f"{type(e).__name__}: {e}"
        res.success, res.message = ok, msg
        res.held = executive.held.item if executive.held else ""
        (goal_handle.succeed if ok else goal_handle.abort)()
        return res

    ActionServer(server_node, ExecuteAction, "/lab_skills/execute", execute,
                 goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.REJECT,
                 callback_group=ReentrantCallbackGroup())
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(server_node)
    log("ready: /lab_skills/execute")
    try:
        ex.spin()
    except KeyboardInterrupt:
        worker.cancel_active()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
