#!/usr/bin/env bash
# Step 2 of 4: the wrist camera (RealSense colour + depth aligned to colour). Keep it running.
#
#   run/2_camera.sh real     the RealSense on the gripper (USB to this computer)
#   run/2_camera.sh sim      the simulated one (start run/1_robot.sh sim first)
#
# Any NAME:=VALUE after that goes to the launch file (camera.launch.py), e.g.
# serial_no:=123456789 or color_profile:=1280,720,30 (real).

source "$(dirname "$0")/env.sh"
shift

if [ "$MODE" = sim ]; then
    exec ros2 launch lab_replica camera.launch.py sim:=true "$@"
fi
exec ros2 launch lab_replica camera.launch.py "$@"
