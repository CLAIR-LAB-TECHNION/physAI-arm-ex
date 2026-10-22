# Lab setup (for the course staff)

How to bring up the robot, the gripper and the camera for the exercise, and
the simulation. Students only run the consoles on their PCs ([README](../README.md)).
The full reference of the earlier workspace is in the README of the `solution` branch.

## Run it in simulation

The simulation needs more packages than a student PC:
`sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ign-ros2-control ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-position-controllers`.
On `main` the consoles stop at the first function that is not written yet;
`git checkout solution` runs the reference solution.

In the clone's top directory:

```console
run/sim.sh                  # terminal 1: wait for "RG6 driver ready (sim)"
run/actions.sh sim          # terminal 2: the action console ...
run/goals.sh sim            #             ... or the goal console
```

* The Gazebo window shows the cell. `run/sim.sh gui:=false` runs without
  it.
* The simulated cell starts with **all twelve items on the table**, the
  basket empty and the gripper empty (the scene
  [`problems/all_on_table`](../problems/all_on_table)). That is what the goal
  console assumes when it starts.
* Another scene: `run/sim.sh t_newcell_1` (any directory name in
  `problems/`, `problems_tamp/` or `problems_gen/`).
* **To start again from the beginning,** stop all terminals (Ctrl-C) and
  start them again from terminal 1. The simulation then starts from its scene
  again.
* The simulation runs slower than real time on the lab PC: an action takes
  30 to 60 s.

## Run it on the real robot

**The lab's set-up (2026-10-18).** Three computers on ROS_DOMAIN_ID 131 (the
lab's allocation for this project):

| Computer | Runs | Needs |
|---|---|---|
| **air** (`ssh air@10.10.34.14`) | 1. the UR driver | `ros-humble-ur` (apt) |
| **air**, second terminal | 3. the RG6 gripper driver | this repository (no build) |
| **guest** (`ssh guest@132.68.35.80`) | 2. the RealSense (plugged in there) | `ros-humble-realsense2-camera` (apt) |
| **this PC** | the consoles (SAM 3 needs its GPU) | this repository, set up with [`run/setup.sh`](../README.md#set-up-once) |

**Before starting any of the drivers,** in every terminal on air and on
guest (their login shells set up neither ROS nor the domain):

```console
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=131
```

### 1. The UR driver (on air)

```console
ssh air@10.10.34.14
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=131
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=10.240.10.4 initial_joint_controller:=passthrough_trajectory_controller
```

Then press **Play** on the pendant (program `guy.urp`). Ready when it prints
`Robot connected to reverse interface. Ready to receive control commands.`

* It is the stock driver from apt (`ros-humble-ur`); it does not use this
  repository.
* `passthrough_trajectory_controller`: the robot interpolates each motion
  itself. With the default (scaled) controller air has to stream a setpoint
  every 2 ms; it misses some (no real-time kernel), the robot drops the
  connection and the pendant program stops (driver log: `Connection to
  reverse interface dropped`). Press Play again after that.

### 2. The RealSense (on guest)

```console
ssh guest@132.68.35.80
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=131
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true rgb_camera.color_profile:=640x360x6 depth_module.depth_profile:=640x360x6
```

Ready when it prints `RealSense Node Is Up!`.

* It is the stock driver from apt (`ros-humble-realsense2-camera`); it does
  not use this repository.
* **640 x 360 at 6 Hz** (instead of the default 1280 x 720 at 30 Hz):
  full-size images do not get across the network. Even these arrive only
  after this PC's UDP receive buffer was raised (needed after every reboot,
  unless made permanent in `/etc/sysctl.d/`):
  ```console
  sudo sysctl -w net.core.rmem_max=2147483647 net.core.rmem_default=33554432 net.ipv4.ipfrag_time=3 net.ipv4.ipfrag_high_thresh=134217728
  ```
  Check: `ros2 topic hz /camera/camera/color/image_raw` on this PC shows
  about 6. 640 x 360 keeps the full field of view (16:9).

### 3. The RG6 gripper driver (on air)

The only driver that needs **this repository**. Once, on air:

```console
git clone https://github.com/guyazran/physAI-arm-ex.git          # any directory
```

Then, each time, in a second terminal on air:

```console
ssh air@10.10.34.14
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=131
cd physAI-arm-ex && git pull          # the clone; same branch as this PC
python3 src/lab_replica/scripts/rg6_driver.py --ros-args -p backend:=onrobot -p ip:=10.240.10.4 -p port:=41414 -p tool_index:=0 -r /joint_states:=/rg6/joint_states
```

Ready when it prints `RG6 driver ready (onrobot)`.

* **No `colcon build` and no `.venv`:** the script puts `src/lab_replica` on
  the Python path and runs from the source tree. It needs only ROS 2 Humble
  (`rclpy`, `sensor_msgs`, `std_msgs`, `control_msgs`; the last comes with
  `ros-humble-ur`).
* The RG6 works through the OnRobot URCap on the robot (XML-RPC on port
  41414 of the robot's address); nothing has to be installed there.
* Its driver runs on air because air is wired to the robot's network. This
  PC reaches it only over Wi-Fi, where the XML-RPC calls time out and the
  console then finds no gripper reading.
* Run only one RG6 driver: not also `real.launch.py` on this PC.

### Then, on this PC

* The three computers' clocks differ by up to two hours. The programs here
  judge whether a camera frame is new by when it arrives, not by its time
  stamp, so this does not matter to them.

**The cell** ([`config/cell.yaml`](../src/lab_replica/config/cell.yaml)): the
slatted table is 1.26 m x 0.85 m, the robot near its back-left corner, the
blue board (0.70 m x 0.59 m, the workspace) to its right. The world frame's
origin is on the table below the robot's base axis, x towards the board, y
towards the wall (the robot's `base_link` is turned 180 degrees from it).
The wall behind the table is an obstacle (`room.wall_y`); the robot may move
anywhere else above and around the table. The basket may stand anywhere on
the board: the robot finds it in every survey.

**Camera pose (hand-eye calibration).** `wrist_camera` in `cell.yaml` was
calibrated on 2026-10-18. Calibrate again if the camera is moved on the
gripper (the scene must stay still; the camera stays 0.55 m or more above
the table):
```console
python3 src/lab_replica/tools/calibrate_wrist_camera.py capture runs/calib_<date>   # moves the arm through 10 views
python3 src/lab_replica/tools/calibrate_wrist_camera.py solve runs/calib_<date>     # prints the new optical_xyz / optical_rpy
```

**Each time** (in the clone's top directory, with `ROS_DOMAIN_ID=131`):

1. Start the UR driver and the RG6 driver on air and the camera on guest
   ([1](#1-the-ur-driver-on-air), [2](#2-the-realsense-on-guest),
   [3](#3-the-rg6-gripper-driver-on-air) above), press **Play** on the pendant.
2. **The scene.** For the goal console: every item on the board, the
   basket empty (anywhere on the board), the gripper empty. Leave at least
   2 cm between items.
3. **On this PC:** the action console **or** the goal console, slowly at
   first:
   ```console
   run/actions.sh real --speed-scale 0.3                  # the action console, or ...
   run/goals.sh real --speed-scale 0.3 --step             # ... the goal console (--step: Enter before every action)
   ```

**Safety.** Keep a hand on the emergency stop. **Ctrl-C** in the console terminal
while an action runs stops the arm and returns to the prompt. Ctrl-C at the
prompt quits. The pendant's speed slider scales every motion, on top of
`--speed-scale`.
