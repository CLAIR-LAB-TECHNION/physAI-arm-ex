#!/usr/bin/env python3
"""ROS 2 node for the RG6 gripper (started by the launch files); see
lab_replica/robot/rg6_driver.py."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from lab_replica.robot.rg6_driver import main  # noqa: E402

if __name__ == "__main__":
    main()
