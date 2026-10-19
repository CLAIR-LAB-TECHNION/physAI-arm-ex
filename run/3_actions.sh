#!/usr/bin/env bash
# Step 3 of 4 (or 4): the action console. Type a PDDL action such as
# (pick-up green-mug); the robot looks, plans and performs it.
#
#   run/3_actions.sh sim
#   run/3_actions.sh real [--speed-scale 0.3]
#
# Needs steps 1 and 2 running. More options: run/3_actions.sh sim --help

source "$(dirname "$0")/env.sh"
shift
use_venv

if [ "$MODE" = sim ]; then
    exec ros2 run lab_replica action_console.py --sim "$@"
fi
exec ros2 run lab_replica action_console.py "$@"
