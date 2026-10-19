#!/usr/bin/env bash
# Step 1 of 4: the robot (UR arm driver + RG6 gripper driver). Keep it running.
#
#   run/1_robot.sh real              the real RG6 (and the UR5e driver if real_robot.yaml has
#                                    ur_driver: true; in the lab it runs on air); Play on the pendant
#   run/1_robot.sh sim [SCENE]       the simulated cell in Gazebo; SCENE is a problem or task
#                                    directory name (default all_on_table: every item on the table)
#
# Any NAME:=VALUE after that goes to the launch file, e.g. gui:=false (sim) or
# robot_ip:=10.240.10.2 (real).

source "$(dirname "$0")/env.sh"
shift

if [ "$MODE" = real ]; then
    exec ros2 launch lab_replica real.launch.py "$@"
fi
scene=all_on_table
if [ $# -gt 0 ] && [[ "$1" != *:=* ]]; then
    scene="$1"
    shift
fi
# The wrist camera is started by run/2_camera.sh sim, as on the real robot.
exec ros2 launch lab_replica sim.launch.py problem:="$scene" wrist_camera:=false "$@"
