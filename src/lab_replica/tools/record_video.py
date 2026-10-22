#!/usr/bin/env python3
"""Record the lab camera to an H.264 MP4 on a simulation-time timeline.

    record_video.py out.mp4 [--fps 30] [--topic /lab_camera/image]

Each frame goes to the slot given by its header stamp (simulation time), so
the video plays at real speed however fast the simulation ran. Missing slots
repeat the previous frame (the count is reported). Stops on SIGINT/SIGTERM.
"""

import argparse
import signal

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--topic", default="/lab_camera/image")
    args = ap.parse_args()
    rclpy.init()
    node = Node("record_video")
    st = {"writer": None, "t0": None, "n": 0, "last": None, "got": 0, "dup": 0, "late": 0}

    def cb(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a[..., ::-1] if m.encoding.lower() in ("rgb8", "r8g8b8") else a
        if st["writer"] is None:
            st["writer"] = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"avc1"), args.fps, (m.width, m.height))
            st["t0"] = t
        slot = int(round((t - st["t0"]) * args.fps))
        if slot < st["n"]:
            st["late"] += 1
            return
        while st["n"] < slot and st["last"] is not None:
            st["writer"].write(st["last"])
            st["n"] += 1
            st["dup"] += 1
        frame = np.ascontiguousarray(bgr)
        st["writer"].write(frame)
        st["n"] += 1
        st["got"] += 1
        st["last"] = frame

    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, args.topic, cb, qos)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    while rclpy.ok() and not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.1)
    if st["writer"] is not None:
        st["writer"].release()
        print(f"wrote {args.out}: {st['n']} frames ({st['n'] / args.fps:.1f} s at {args.fps:g} fps), "
              f"{st['got']} rendered, {st['dup']} repeated, {st['late']} out of order")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
