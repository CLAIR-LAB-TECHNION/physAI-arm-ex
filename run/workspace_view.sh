#!/usr/bin/env bash
# Find the observation configuration (OBSERVATION_JOINTS in
# exercise/state_estimation.py): a window with the wrist camera live and
# SAM 3's masks of the items, and the arm's joints in this terminal as they
# change. Move the arm by hand (freedrive) and copy the joints. Never moves
# the arm itself.
#
#   run/workspace_view.sh sim
#   run/workspace_view.sh real [--min-score 0.3]
#
# Needs steps 1 and 2 running; instead of step 3. More options:
# run/workspace_view.sh sim --help

source "$(dirname "$0")/env.sh"
shift
use_venv

if [ "$MODE" = sim ]; then
    exec ros2 run lab_replica workspace_view.py --sim "$@"
fi
exec ros2 run lab_replica workspace_view.py "$@"
