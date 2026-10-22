"""The consoles' camera window: the wrist RealSense live, colour and
depth side by side, and below it every observation the skills make,
annotated (autonomy/annotate.py: SAM 3 masks and the estimated items, the
planned grasp, where an item is put or dropped, a new item's measurement).

The window runs in a child process, so it keeps redrawing while the console
waits for input or plans. The console sends it the images through queues:
no second ROS subscription, so the camera's images cross the network to
this computer once (on the real robot they barely fit, README).

Keys in the window: a / left arrow and d / right arrow browse the
observations, l / End jumps back to the newest.
"""

import multiprocessing as mp
import queue
import time

import cv2
import numpy as np

TITLE = "wrist camera: live and observations"
PANEL_W, PANEL_H = 640, 360        # each of the four panels
LABEL_H, BAR_H = 26, 54
BEYOND_MEDIAN = 0.08               # m: the depth colours end this far past the median (mostly the table)


def depth_range(depth):
    """Colour scale [lo, hi] for a depth image: from its nearest points to a
    little beyond the median, so the items on the table spread over the
    colours and the floor beyond the table is simply far (blue)."""
    v = depth[np.isfinite(depth)]
    if not v.size:
        return 0.2, 1.2
    lo, mid, far = np.percentile(v, [2, 50, 98])
    return float(lo), float(max(lo + 0.05, min(far, mid + BEYOND_MEDIAN)))


def depth_colors(depth, lo=None, hi=None):
    """(BGR image, lo, hi): depth in metres coloured from near (red) to far
    (blue) over [lo, hi] (default: depth_range), unknown black."""
    ok = np.isfinite(depth)
    if lo is None or hi is None:
        lo, hi = depth_range(depth)
    t = np.zeros(depth.shape, np.uint8)
    t[ok] = (255 * (1.0 - np.clip((depth[ok] - lo) / (hi - lo), 0.0, 1.0))).astype(np.uint8)
    img = cv2.applyColorMap(t, cv2.COLORMAP_TURBO)
    img[~ok] = 0
    return img, lo, hi


def depth_legend(img, lo, hi):
    """A colour bar with its range in the bottom-right corner of `img` (BGR)."""
    H, W = img.shape[:2]
    s = W / 640.0
    bw, bh = int(110 * s), int(10 * s)
    x0, y0 = W - bw - int(12 * s), H - int(30 * s)
    bar = cv2.applyColorMap(np.linspace(255, 0, bw).astype(np.uint8)[None, :].repeat(bh, 0), cv2.COLORMAP_TURBO)
    img[y0:y0 + bh, x0:x0 + bw] = bar
    cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + bw, y0 + bh), (255, 255, 255), 1)
    for x, v in ((x0, lo), (x0 + bw, hi)):
        _text(img, f"{v:.2f} m", (x - int(22 * s), y0 + bh + int(16 * s)), 0.4 * s, 1)
    return img


def _text(img, text, org, scale=0.55, thick=1, color=(255, 255, 255)):
    org = (int(org[0]), int(org[1]))
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def _fit(img, w, h):
    """`img` scaled into w x h (aspect kept, black borders)."""
    out = np.zeros((h, w, 3), np.uint8)
    if img is None:
        return out
    s = min(w / img.shape[1], h / img.shape[0])
    iw, ih = max(1, int(img.shape[1] * s)), max(1, int(img.shape[0] * s))
    small = cv2.resize(img, (iw, ih), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    y, x = (h - ih) // 2, (w - iw) // 2
    out[y:y + ih, x:x + iw] = small
    return out


# ---- the console's side ---------------------------------------------------------------------

class Viewer:
    """Starts the window process; its methods never block the console."""

    def __init__(self, title=TITLE):
        ctx = mp.get_context("spawn")   # not fork: the console has CUDA (SAM 3) and ROS threads
        self.live_q = ctx.Queue(maxsize=2)
        self.event_q = ctx.Queue(maxsize=200)
        # On exit, do not wait to flush images the window will never read.
        self.live_q.cancel_join_thread()
        self.event_q.cancel_join_thread()
        self.proc = ctx.Process(target=_window, args=(self.live_q, self.event_q, title), daemon=True,
                                name="camera-window")
        self.proc.start()
        self.last_live = 0.0

    def alive(self):
        return self.proc.is_alive()

    def live(self, rgb, depth, max_rate=10.0):
        """A live frame (RGB uint8, depth float metres); dropped when the
        window is busy or frames come faster than max_rate."""
        now = time.time()
        if now - self.last_live < 1.0 / max_rate or not self.alive():
            return
        self.last_live = now
        try:
            self.live_q.put_nowait((rgb, depth))
        except queue.Full:
            pass

    def _event(self, ev):
        if not self.alive():
            return
        try:
            self.event_q.put_nowait(ev)
        except queue.Full:
            pass

    def observation(self, rgb_vis, depth_vis, title):
        """An annotated observation: two BGR images (colour, depth) and its title."""
        self._event(("observation", rgb_vis, depth_vis, title))

    def status(self, text):
        self._event(("status", text))

    def close(self):
        if self.alive():
            try:
                self.event_q.put(None, timeout=1.0)
            except queue.Full:
                pass
            self.proc.join(3.0)
        if self.proc.is_alive():
            self.proc.terminate()


# ---- the window process ---------------------------------------------------------------------

KEYS_PREV = {ord("a"), ord("A"), 65361, 2424832}    # a, left arrow (GTK, Windows)
KEYS_NEXT = {ord("d"), ord("D"), 65363, 2555904}    # d, right arrow
KEYS_LAST = {ord("l"), ord("L"), 65367, 2293760}    # l, End


def _window(live_q, event_q, title):
    cv2.namedWindow(title, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
    W = 2 * PANEL_W
    Htot = 2 * (LABEL_H + PANEL_H) + BAR_H
    cv2.resizeWindow(title, W, Htot)
    live = None                 # (rgb BGR panel, depth panel), composed
    live_range = None           # colour scale of the live depth, smoothed
    obs = []                    # [(colour, depth, title, time)]
    index = None                # browsed observation; None: the newest
    status = "waiting for the first action"
    dirty = True
    shown = mapped = False
    while True:
        try:
            while True:
                ev = event_q.get_nowait()
                if ev is None:
                    cv2.destroyAllWindows()
                    return
                if ev[0] == "observation":
                    obs.append((ev[1], ev[2], ev[3], time.strftime("%H:%M:%S")))
                elif ev[0] == "status":
                    status = ev[1]
                dirty = True
        except queue.Empty:
            pass
        frame = None
        try:
            while True:
                frame = live_q.get_nowait()
        except queue.Empty:
            pass
        if frame is not None:
            rgb, depth = frame
            r = np.array(depth_range(depth))
            live_range = r if live_range is None else 0.8 * live_range + 0.2 * r
            dimg, lo, hi = depth_colors(depth, *live_range)
            live = (_fit(np.ascontiguousarray(rgb[:, :, ::-1]), PANEL_W, PANEL_H),
                    depth_legend(_fit(dimg, PANEL_W, PANEL_H), lo, hi))
            dirty = True
        if dirty:
            canvas = np.zeros((Htot, W, 3), np.uint8)
            y = 0
            _label_row(canvas, y, "LIVE: colour", "LIVE: depth (aligned to the colour)")
            y += LABEL_H
            if live is not None:
                canvas[y:y + PANEL_H, :PANEL_W], canvas[y:y + PANEL_H, PANEL_W:] = live
            else:
                _text(canvas, "no camera image yet", (W // 2 - 110, y + PANEL_H // 2), 0.7, 2)
            y += PANEL_H
            k = len(obs) - 1 if index is None else index
            if obs:
                c, d, t, at = obs[k]
                _label_row(canvas, y, f"OBSERVATION {k + 1}/{len(obs)} at {at}: {t}", "its depth")
                y += LABEL_H
                canvas[y:y + PANEL_H, :PANEL_W] = _fit(c, PANEL_W, PANEL_H)
                canvas[y:y + PANEL_H, PANEL_W:] = _fit(d, PANEL_W, PANEL_H)
            else:
                _label_row(canvas, y, "OBSERVATIONS", "")
                y += LABEL_H
                _text(canvas, "each view the robot takes appears here, annotated",
                      (W // 2 - 270, y + PANEL_H // 2), 0.7, 2)
            y += PANEL_H
            cv2.rectangle(canvas, (0, y), (W, Htot), (30, 30, 30), -1)
            _text(canvas, status[:110], (12, y + 22), 0.6, 1, (120, 220, 255))
            hint = "a / d: earlier / later observation   l: newest" + ("" if index is None else "   (browsing)")
            _text(canvas, hint, (12, y + 45), 0.45, 1, (190, 190, 190))
            cv2.imshow(title, canvas)
            dirty = False
            shown = True
        key = cv2.waitKeyEx(30)
        if key != -1 and obs:
            k = len(obs) - 1 if index is None else index
            if key in KEYS_PREV:
                index = max(0, k - 1)
            elif key in KEYS_NEXT:
                index = None if k + 1 >= len(obs) - 1 else k + 1
            elif key in KEYS_LAST:
                index = None
            dirty = True
        if shown:
            # Closed by the user (once it has been on screen: right after
            # imshow GTK may not have mapped it yet): the console carries on.
            try:
                visible = cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) >= 1
            except cv2.error:
                visible = False
            if visible:
                mapped = True
            elif mapped:
                return


def _label_row(canvas, y, left, right):
    canvas[y:y + LABEL_H] = (45, 45, 45)
    _text(canvas, left[:70], (8, y + 18), 0.5, 1)
    _text(canvas, right[:70], (PANEL_W + 8, y + 18), 0.5, 1)
