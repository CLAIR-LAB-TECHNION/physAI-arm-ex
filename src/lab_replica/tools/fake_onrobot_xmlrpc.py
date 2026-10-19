#!/usr/bin/env python3
"""Stand-in for the OnRobot URCap daemon (XML-RPC, port 41414) for testing
rg6_driver.py's 'onrobot' backend (real.launch.py) without the robot.

    python3 fake_onrobot_xmlrpc.py [--port 41414] [--object-mm 58]

Serves the calls the driver uses: get_discovery, rg_grip, rg_stop,
rg_get_width, rg_get_busy, rg_get_grip_detected, rg_get_status (and
system.multicall). The fingers move at 100 mm/s and stop on an object of
--object-mm width when closing on it (grip detected).
"""

import argparse
import json
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer

SPEED = 100.0  # mm/s


class FakeRG6:
    def __init__(self, object_mm):
        self.object_mm = object_mm
        self.width = 88.7
        self.target = self.width
        self.grip = False
        self.t = time.time()
        self.lock = threading.Lock()
        self.log = []

    def _update(self):
        now = time.time()
        step = SPEED * (now - self.t)
        self.t = now
        lo = self.object_mm if self.target < self.object_mm <= self.width + 1e-6 else -1.0
        d = self.target - self.width
        self.width += max(-step, min(step, d))
        if lo > 0 and self.width <= lo:
            self.width, self.target, self.grip = lo, lo, True

    def busy(self):
        return abs(self.target - self.width) > 1e-6

    # XML-RPC methods
    def get_discovery(self):
        return json.dumps({"devices": [{"deviceName": "RG6 (fake)", "status": 0}]})

    def rg_grip(self, index, width, force):
        with self.lock:
            self._update()
            self.log.append(("rg_grip", index, width, force))
            print(f"rg_grip(index={index}, width={width}, force={force})", flush=True)
            self.target, self.grip = float(width), False
            return 0

    def rg_stop(self, index):
        with self.lock:
            self._update()
            print("rg_stop", flush=True)
            self.target = self.width
            return 0

    def rg_get_width(self, index):
        with self.lock:
            self._update()
            return self.width

    def rg_get_busy(self, index):
        with self.lock:
            self._update()
            return self.busy()

    def rg_get_grip_detected(self, index):
        with self.lock:
            self._update()
            return self.grip

    def rg_get_status(self, index):
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=41414)
    ap.add_argument("--object-mm", type=float, default=58.0)
    args = ap.parse_args()
    srv = SimpleXMLRPCServer(("127.0.0.1", args.port), logRequests=False, allow_none=True)
    srv.register_multicall_functions()
    srv.register_instance(FakeRG6(args.object_mm))
    print(f"fake OnRobot daemon on 127.0.0.1:{args.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
