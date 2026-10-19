#!/usr/bin/env bash
# Step 4 of 4 (or 3): the goal console. Type a goal such as
# (in-basket green-mug) (in-basket red-can); the robot plans the actions
# with Fast Downward and performs them one after the other. It assumes that
# every item is on the table and the gripper is empty when it starts.
#
#   run/4_goals.sh sim
#   run/4_goals.sh real [--speed-scale 0.3] [--step]
#
# Needs steps 1 and 2 running. More options: run/4_goals.sh sim --help

source "$(dirname "$0")/env.sh"
shift
use_venv

if [ "$MODE" = sim ]; then
    exec ros2 run lab_replica goal_console.py --sim "$@"
fi
exec ros2 run lab_replica goal_console.py "$@"
