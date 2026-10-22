#!/usr/bin/env bash
# The goal console. Type a goal such as (in-basket green-mug) (in-basket
# red-can); the robot plans the actions (UPF with Fast Downward) and performs
# them one after the other. It assumes that every item is on the table and the
# gripper is empty when it starts.
#
#   run/goals.sh sim
#   run/goals.sh real [--speed-scale 0.3] [--step]
#
# Needs run/sim.sh (sim) or the robot's drivers and camera (real) running.
# More options: run/goals.sh sim --help

source "$(dirname "$0")/env.sh"
read_mode "$@"
shift
banner
use_venv

if [ "$MODE" = sim ]; then
    exec ros2 run lab_replica goal_console.py --sim "$@"
fi
exec ros2 run lab_replica goal_console.py "$@"
