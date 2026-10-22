# t_newcell_1

<!-- Written by solve.py. Edits are overwritten. -->

A task for the autonomous skills: the goal below (from the command line), in the
scene generated with seed 11.

**Initial state:** 11 items on the table, (in-basket yellow-block), (robot-gripper-empty)

**Goal:** `(and (in-basket yellow-block) (in-basket red-can))`

**Plan (Fast Downward, 2 actions):**

1. (pick-up red-can)
2. (put-in-basket red-can)

## Run it

All in one (starts the simulation and the skill server, executes the plan,
checks the goal in the simulator):

```console
ros2 run lab_replica solve.py --task t_newcell_1
```

With the simulation (or the real robot) and the skill server already
running (README section T.2):

```console
ros2 launch lab_replica sim.launch.py problem:=t_newcell_1
ros2 run lab_replica skill_server.py --sim
ros2 run lab_replica solve.py --task t_newcell_1 --attach
```

## Setting it up on the real table

Place the items as in [`layout.png`](layout.png) and the table below (world
frame: origin on the table surface under the robot base axis, x to the right
as seen from the camera, y towards the wall; yaw about the vertical from +x).
The basket's centre is at (55.2, 1.2) cm. The robot finds the items
itself, so a few centimetres off is fine.

| # | Item | Where | Resting pose | x [cm] | y [cm] | yaw [deg] |
|---|---|---|---|---|---|---|
| 1 | blue-block | table | flat | 37.3 | 21.4 | -39 |
| 2 | blue-cup-with-yellow-stripe | table | upright | 19.3 | -19.3 | 4 |
| 3 | green-block | table | flat | 65.4 | -23.0 | -69 |
| 4 | green-mug | table | upright | 30.7 | -8.5 | -34 |
| 5 | orange-block | table | flat | 67.1 | 21.6 | 80 |
| 6 | red-block | table | flat | 15.9 | 17.3 | -166 |
| 7 | red-can | table | upright | 20.6 | -10.3 | 4 |
| 8 | red-spray-bottle | table | upright | 27.3 | 3.9 | 115 |
| 9 | roll-of-black-tape | table | flat | 25.9 | 19.2 | -157 |
| 10 | roll-of-blue-tape | table | flat | 30.0 | -20.1 | 128 |
| 11 | roll-of-green-tape | table | flat | 42.1 | -21.5 | -176 |
| 12 | yellow-block | basket | flat | 53.9 | -2.4 | -132 |
