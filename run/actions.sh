#!/usr/bin/env bash
# The action console. Type a PDDL action such as (pick-up green-mug); the
# robot looks, plans and performs it.
#
#   run/actions.sh sim
#   run/actions.sh real [--speed-scale 0.3]
#
# Needs run/sim.sh (sim) or the robot's drivers and camera (real) running.
# More options: run/actions.sh sim --help

source "$(dirname "$0")/env.sh"
read_mode "$@"
shift
banner
use_venv

if [ "$MODE" = sim ]; then
    exec ros2 run lab_replica action_console.py --sim "$@"
fi
exec ros2 run lab_replica action_console.py "$@"
