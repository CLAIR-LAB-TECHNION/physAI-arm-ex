#!/usr/bin/env bash
# The simulated cell in Gazebo: the arm, the gripper and the wrist camera.
# Keep it running; start a console in another terminal (sim). Only for the
# simulation: on the real robot the drivers run on other computers.
#
#   run/sim.sh [SCENE]
#
# SCENE is a problem or task directory name (default all_on_table: every item
# on the table). Any NAME:=VALUE after that goes to sim.launch.py, e.g.
# gui:=false (no Gazebo window).

source "$(dirname "$0")/env.sh"
MODE=sim
banner

scene=all_on_table
if [ $# -gt 0 ] && [[ "$1" != *:=* ]]; then
    scene="$1"
    shift
fi
exec ros2 launch lab_replica sim.launch.py problem:="$scene" "$@"
