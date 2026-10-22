# t_newcell_2

<!-- Written by solve.py. Edits are overwritten. -->

A task for the autonomous skills: the goal below (from the command line), in the
scene generated with seed 21.

**Initial state:** 12 items on the table, (robot-gripper-empty)

**Goal:** `(and (in-basket red-spray-bottle) (robot-gripping yellow-block))`

**Plan (Fast Downward, 3 actions):**

1. (pick-up red-spray-bottle)
2. (put-in-basket red-spray-bottle)
3. (pick-up yellow-block)

## Run it

All in one (starts the simulation and the skill server, executes the plan,
checks the goal in the simulator):

```console
ros2 run lab_replica solve.py --task t_newcell_2
```

With the simulation (or the real robot) and the skill server already
running (README section T.2):

```console
ros2 launch lab_replica sim.launch.py problem:=t_newcell_2
ros2 run lab_replica skill_server.py --sim
ros2 run lab_replica solve.py --task t_newcell_2 --attach
```

## Setting it up on the real table

Place the items as in [`layout.png`](layout.png) and the table below (world
frame: origin on the table surface under the robot base axis, x to the right
as seen from the camera, y towards the wall; yaw about the vertical from +x).
The basket's centre is at (63.0, 6.7) cm. The robot finds the items
itself, so a few centimetres off is fine.

| # | Item | Where | Resting pose | x [cm] | y [cm] | yaw [deg] |
|---|---|---|---|---|---|---|
| 1 | blue-block | table | flat | 24.7 | 4.7 | 52 |
| 2 | blue-cup-with-yellow-stripe | table | upright | 41.7 | -14.4 | 83 |
| 3 | green-block | table | flat | 35.1 | 22.1 | 64 |
| 4 | green-mug | table | upright | 25.1 | -7.8 | 51 |
| 5 | orange-block | table | flat | 12.9 | 19.8 | 172 |
| 6 | red-block | table | flat | 15.0 | -21.4 | 0 |
| 7 | red-can | table | upright | 23.1 | -18.3 | 51 |
| 8 | red-spray-bottle | table | upright | 35.2 | 7.8 | 90 |
| 9 | roll-of-black-tape | table | flat | 22.3 | 16.7 | -113 |
| 10 | roll-of-blue-tape | table | flat | 50.2 | -21.5 | -160 |
| 11 | roll-of-green-tape | table | flat | 61.0 | -21.6 | 89 |
| 12 | yellow-block | table | flat | 30.1 | -20.8 | 171 |
