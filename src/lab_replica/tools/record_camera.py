#!/usr/bin/env python3
"""Record the lab camera to an animated GIF (and keep the last frame as PNG).

    record_camera.py out.gif [--period 1.0] [--scale 0.5] [--speedup 4]

Stops on Ctrl-C or SIGTERM and writes the file.
"""

import argparse
import signal
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--period", type=float, default=1.0, help="wall seconds between kept frames")
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--speedup", type=float, default=4.0, help="playback speed relative to wall time")
    args = ap.parse_args()
    rclpy.init()
    node = Node("record_camera")
    frames = []
    last = [0.0]

    def cb(m):
        now = time.time()
        if now - last[0] < args.period:
            return
        last[0] = now
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        img = PILImage.fromarray(a.copy(), "RGB")
        img = img.resize((int(m.width * args.scale), int(m.height * args.scale)))
        frames.append(img)

    node.create_subscription(Image, "/lab_camera/image", cb, 2)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    try:
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if frames:
        dur = int(1000 * args.period / args.speedup)
        frames[0].save(args.out, save_all=True, append_images=frames[1:], duration=dur, loop=0, optimize=True)
        frames[-1].save(args.out.rsplit(".", 1)[0] + "_last.png")
        print(f"wrote {args.out} ({len(frames)} frames)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
