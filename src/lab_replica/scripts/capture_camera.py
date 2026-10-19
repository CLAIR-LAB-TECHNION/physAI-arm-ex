#!/usr/bin/env python3
"""Save one frame of the simulated lab camera (/lab_camera/image) as a PNG.

    ros2 run lab_replica capture_camera.py out.png [--topic /lab_camera/image] [--timeout 20]
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--topic", default="/lab_camera/image")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = Node("capture_camera")
    got = {}

    def cb(msg):
        got["img"] = msg

    node.create_subscription(Image, args.topic, cb, 1)
    t0 = time.time()
    while rclpy.ok() and "img" not in got and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "img" not in got:
        print("no image received on", args.topic, file=sys.stderr)
        sys.exit(1)
    m = got["img"]
    arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
    from PIL import Image as PILImage

    if m.encoding.lower() == "bgr8":
        arr = arr[..., ::-1]
    PILImage.fromarray(arr, "RGB").save(args.out)
    print("saved", args.out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
