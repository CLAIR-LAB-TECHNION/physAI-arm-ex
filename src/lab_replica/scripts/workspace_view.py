#!/usr/bin/env python3
"""Find the observation configuration: what the wrist camera sees from
where the arm stands, and the arm's joints.

    run/workspace_view.sh sim|real               (the usual way: sets up the environment)
    ros2 run lab_replica workspace_view.py [--min-score S]

Needs the arm (only its joint states: it is never moved from here) and the
wrist camera (run/sim.sh, or on the real robot their drivers: README, "Run it
on the real robot"). A window shows

    top:     the camera live, colour and depth side by side
    bottom:  the same images with SAM 3's masks of the items and the basket
             (config/perception.yaml prompts), as fast as SAM 3 keeps up

and the terminal shows the arm's joints as they change, as a line to copy
into exercise/state_estimation.py (OBSERVATION_JOINTS). Move the arm by hand
(freedrive on the teach pendant) until the camera sees the whole board and
SAM 3 finds every item and the basket.

Keys in the window: s prints the joints on a line of their own (to keep),
q or Esc quits (so does closing the window, or Ctrl-C).
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

TITLE = "wrist camera: live and SAM 3"


class Camera:
    """The newest colour + depth images and arm joints (spun in a thread)."""

    def __init__(self, node, cell, cfg):
        from sensor_msgs.msg import JointState

        from lab_replica.perception.rgbd import RGBDCamera
        from lab_replica.toolkit import ARM_JOINTS

        self.lock = threading.Lock()
        self.image = None       # (rgb, depth, arrival time)
        self.q = None
        self.cam = RGBDCamera(node, cell["robot"]["ur_type"], cfg["topics"])
        self.cam.on_image = self._on_image

        def on_joints(msg):
            d = dict(zip(msg.name, msg.position))
            if all(j in d for j in ARM_JOINTS):
                with self.lock:
                    self.q = np.array([d[j] for j in ARM_JOINTS])

        node.create_subscription(JointState, "/joint_states", on_joints, 50)

    def _on_image(self, color_msg, depth_msg):
        from lab_replica.perception.rgbd import decode_image
        rgb, depth = decode_image(color_msg), decode_image(depth_msg)
        if rgb.shape[:2] != depth.shape:
            return
        with self.lock:
            self.image = (rgb, depth, time.time())

    def newest(self):
        with self.lock:
            return self.image, None if self.q is None else self.q.copy()

    def K(self):
        info = self.cam.info
        return None if info is None else np.array(info.k, float).reshape(3, 3)


class Segmenter(threading.Thread):
    """SAM 3 on the newest image, again and again; keeps the last result."""

    def __init__(self, camera, cfg, min_score):
        super().__init__(daemon=True, name="sam3")
        self.camera, self.cfg, self.min_score = camera, cfg, min_score
        self.prompts = {p: item for item, ps in cfg["prompts"].items() for p in ps}
        self.basket_prompt = cfg["basket_prompt"]
        self.lock = threading.Lock()
        self.result = None      # (colour BGR, depth BGR, found, missing, seconds per image, when the image arrived)
        self.state = "loading SAM 3 (the segmentation model)"
        self.stop = threading.Event()

    def run(self):
        os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
        os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")
        import warnings
        warnings.filterwarnings("ignore", category=FutureWarning)
        try:
            from lab_replica.perception.sam3 import Sam3
            sam = Sam3(self.cfg["sam3"]["model"])
        except Exception as e:  # noqa: BLE001
            self.state = f"SAM 3 did not load: {e}"
            return
        self.state = "waiting for a camera image"
        last = None
        while not self.stop.is_set():
            image, _ = self.camera.newest()
            K = self.camera.K()
            if image is None or K is None or image[2] == last:
                time.sleep(0.02)
                continue
            last = image[2]
            t0 = time.time()
            rgb, depth, _ = image
            res = sam.segment(rgb, list(self.prompts) + [self.basket_prompt],
                              threshold=min(self.min_score, self.cfg["sam3"]["threshold"]))
            found = self.masks(res)
            colour, depth_vis = self.annotate(rgb, depth, K, found)
            names = [k for k, _, _ in found]
            missing = [i for i in list(self.cfg["prompts"]) + ["basket"] if i not in names]
            with self.lock:
                self.result = (colour, depth_vis, names, missing, time.time() - t0, last)

    def masks(self, res):
        """[(name, score, mask)]: greedily, best first, one mask per item
        (and for the basket) with at least min_score, none covering mostly
        the pixels of a mask already taken (the basket aside: items stand in it)."""
        from lab_replica.toolkit import mask_overlap
        cands = sorted(((inst.score, item, inst.mask)
                        for p, item in list(self.prompts.items()) + [(self.basket_prompt, "basket")]
                        for inst in res[p] if inst.score >= self.min_score), key=lambda c: -c[0])
        kept = []
        for score, item, mask in cands:
            if any(k == item for k, _, _ in kept):
                continue
            if item == "basket" or all(k == "basket" or mask_overlap(mask, m) < 0.5 for k, _, m in kept):
                kept.append((item, score, mask))
        return kept

    def annotate(self, rgb, depth, K, found):
        """The masks drawn on the colour image and on the depth (BGR both)."""
        from lab_replica.autonomy.annotate import DepthLook, item_color, label
        from lab_replica.perception.rgbd import Frame
        from lab_replica.viewer import depth_legend

        frame = Frame(rgb, depth, K, 0.0)
        look = DepthLook(frame)
        out = []
        for view in (frame, look):
            vis = view.rgb[:, :, ::-1].copy()
            for item, score, m in found:
                col = item_color(item)
                vis[m] = (0.5 * vis[m] + 0.5 * np.array(col)).astype(np.uint8)
                cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(vis, cnts, -1, col, 2)
                v, u = np.nonzero(m)
                label(vis, f"{item} {score:.2f}", (int(u.mean()) + 8, int(v.mean()) - 8), col,
                      scale=vis.shape[1] / 1500.0)
            out.append(vis)
        return out[0], depth_legend(out[1], look.lo, look.hi)

    def newest(self):
        with self.lock:
            return self.result


# ---- the terminal ---------------------------------------------------------------------------

class Joints:
    """The arm's joints in the terminal, rewritten in place as they change."""

    def __init__(self, camera_pose):
        self.pose = camera_pose     # arm joints -> the camera's optical frame in the world
        self.tty = sys.stdout.isatty()
        self.lines = 0          # lines of the block on screen
        self.last = None
        self.last_print = 0.0

    def text(self, q):
        if q is None:
            return ["arm: no /joint_states yet (is the robot running?)"]
        T = self.pose(q)
        tilt = np.degrees(np.arccos(np.clip(-T[2, 2], -1.0, 1.0)))
        return ["OBSERVATION_JOINTS = [" + ", ".join(f"{v:.4f}" for v in q) + "]",
                "  deg: [" + ", ".join(f"{v:.1f}" for v in np.degrees(q)) + "]",
                f"  camera at ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}) m, "
                f"looking {tilt:.1f} deg from straight down"]

    def show(self, q):
        lines = self.text(q)
        if self.tty:
            if lines == self.last:
                return
            out = (f"\x1b[{self.lines}F" if self.lines else "") + "".join(f"\x1b[2K{t}\n" for t in lines)
            sys.stdout.write(out + "\x1b[J")
            self.lines = len(lines)
        elif lines != self.last and time.time() - self.last_print > 2.0:
            # Not a terminal (a log file): a line now and then, not a flood.
            print(lines[0])
            self.last_print = time.time()
        else:
            return
        self.last = lines
        sys.stdout.flush()

    def keep(self, q):
        """The joints on a line of their own above the live block."""
        if self.tty and self.lines:
            sys.stdout.write(f"\x1b[{self.lines}F\x1b[J")
            self.lines, self.last = 0, None
        print(f"[{time.strftime('%H:%M:%S')}] " + (self.text(q)[0] if q is not None else "no joints yet"))


# ---- the window -----------------------------------------------------------------------------

BAR_H = 76


def compose(live, seg, seg_state, status, n_items, min_score):
    """The window's image: the live images, SAM 3's below, a status bar."""
    from lab_replica.viewer import LABEL_H, PANEL_H, PANEL_W, _fit, _label_row, _text
    W, H = 2 * PANEL_W, 2 * (LABEL_H + PANEL_H) + BAR_H
    canvas = np.zeros((H, W, 3), np.uint8)
    _label_row(canvas, 0, "LIVE: colour", "LIVE: depth (aligned to the colour)")
    y = LABEL_H
    if live is not None:
        canvas[y:y + PANEL_H, :PANEL_W], canvas[y:y + PANEL_H, PANEL_W:] = live
    else:
        _text(canvas, "no camera image yet (is the camera running?)", (W // 2 - 260, y + PANEL_H // 2), 0.7, 2)
    y += PANEL_H
    missing = None
    if seg is not None:
        colour, depth, names, missing, dt, at = seg
        n = len([k for k in names if k != "basket"])
        _label_row(canvas, y, f"SAM 3 (score >= {min_score:.2f}): {n} of {n_items} items, "
                              + ("the basket" if "basket" in names else "NO basket"),
                   f"its depth   (image of {time.strftime('%H:%M:%S', time.localtime(at))}, {dt:.1f} s per image)")
        canvas[y + LABEL_H:y + LABEL_H + PANEL_H, :PANEL_W] = _fit(colour, PANEL_W, PANEL_H)
        canvas[y + LABEL_H:y + LABEL_H + PANEL_H, PANEL_W:] = _fit(depth, PANEL_W, PANEL_H)
    else:
        _label_row(canvas, y, "SAM 3", "")
        _text(canvas, seg_state, (W // 2 - 220, y + LABEL_H + PANEL_H // 2), 0.7, 2)
    y += LABEL_H + PANEL_H
    cv2.rectangle(canvas, (0, y), (W, H), (30, 30, 30), -1)
    _text(canvas, status[:110], (12, y + 22), 0.6, 1, (120, 220, 255))
    if missing is not None:
        _text(canvas, ("not found: " + ", ".join(missing) if missing else "every item and the basket found")[:120],
              (12, y + 45), 0.5, 1, (90, 170, 255) if missing else (120, 230, 120))
    _text(canvas, "s: print the joints to keep   q / Esc: quit", (12, y + 67), 0.45, 1, (190, 190, 190))
    return canvas


def live_panels(image, live_range):
    from lab_replica.viewer import PANEL_H, PANEL_W, _fit, depth_colors, depth_legend, depth_range
    rgb, depth, _ = image
    r = np.array(depth_range(depth))
    live_range = r if live_range is None else 0.8 * live_range + 0.2 * r
    dimg, lo, hi = depth_colors(depth, *live_range)
    return ((_fit(np.ascontiguousarray(rgb[:, :, ::-1]), PANEL_W, PANEL_H),
             depth_legend(_fit(dimg, PANEL_W, PANEL_H), lo, hi)), live_range)


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="the simulation (its clock)")
    ap.add_argument("--min-score", type=float, default=0.3,
                    help="show SAM 3's masks with at least this score (default 0.3)")
    args, _ = ap.parse_known_args()
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise SystemExit("no display: run it on the lab PC's screen, or over ssh -X")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions

    from lab_replica import config
    from lab_replica.perception.scene import load_perception_config

    cell, cfg = config.load_cell(), load_perception_config()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl-C: KeyboardInterrupt here
    node = rclpy.create_node("lab_workspace_view")
    if args.sim:
        node.set_parameters([Parameter("use_sim_time", value=True)])
    camera = Camera(node, cell, cfg)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True, name="ros")
    spinner.start()
    seg = Segmenter(camera, cfg, args.min_score)
    seg.start()
    joints = Joints(camera.cam.pose)

    print(f"Camera: {cfg['topics']['color']}, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}")
    print("The arm's joints [rad], for exercise/state_estimation.py (s in the window keeps a line, q quits):")
    cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
    canvas = compose(None, None, seg.state, "", 0, 0)
    cv2.resizeWindow(TITLE, canvas.shape[1], canvas.shape[0])
    live, live_range, live_at = None, None, None
    mapped = False
    try:
        while True:
            image, q = camera.newest()
            joints.show(q)
            if image is not None and image[2] != live_at:
                live_at = image[2]
                live, live_range = live_panels(image, live_range)
            result = seg.newest()
            status = "arm: no joint states yet" if q is None else \
                "q = [" + ", ".join(f"{v:.3f}" for v in q) + "] rad"
            if image is not None and time.time() - image[2] > 2.0:
                status += f"   camera: no image for {time.time() - image[2]:.0f} s"
            cv2.imshow(TITLE, compose(live, result, seg.state, status, len(cfg["prompts"]), args.min_score))
            key = cv2.waitKeyEx(50)
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):
                joints.keep(q)
            try:
                visible = cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE) >= 1
            except cv2.error:
                visible = False
            if visible:
                mapped = True
            elif mapped:
                break
    except KeyboardInterrupt:
        pass
    finally:
        image, q = camera.newest()
        joints.keep(q)
        seg.stop.set()
        cv2.destroyAllWindows()
        executor.shutdown(timeout_sec=1.0)
        spinner.join(2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        seg.join(5.0)
        if seg.is_alive():
            # Still loading SAM 3: a thread left in torch aborts the exit.
            sys.stdout.flush()
            os._exit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
