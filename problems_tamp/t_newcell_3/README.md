# t_newcell_3

<!-- Written by solve.py. Edits are overwritten. -->

A task for the autonomous skills: the goal below (from the command line), in the
scene generated with seed 31.

**Initial state:** 10 items on the table, (in-basket roll-of-black-tape), (robot-gripping red-block)

**Goal:** `(and (in-basket blue-cup-with-yellow-stripe) (in-basket green-mug))`

**Plan (Fast Downward, 5 actions):**

1. (put-in-basket red-block)
2. (pick-up blue-cup-with-yellow-stripe)
3. (put-in-basket blue-cup-with-yellow-stripe)
4. (pick-up green-mug)
5. (put-in-basket green-mug)

## Run it

All in one (starts the simulation and the skill server, executes the plan,
checks the goal in the simulator):

```console
ros2 run lab_replica solve.py --task t_newcell_3
```

With the simulation (or the real robot) and the skill server already
running (README section T.2):

```console
ros2 launch lab_replica sim.launch.py problem:=t_newcell_3
ros2 run lab_replica skill_server.py --sim
ros2 run lab_replica solve.py --task t_newcell_3 --attach
```

The initial state has the robot holding the **red-block**. Before the plan, the robot picks it up from its place on the table (skipped when the gripper already holds something).

## Setting it up on the real table

Place the items as in [`layout.png`](layout.png) and the table below (world
frame: origin on the table surface under the robot base axis, x to the right
as seen from the camera, y towards the wall; yaw about the vertical from +x).
The basket's centre is at (55.4, -1.1) cm. The robot finds the items
itself, so a few centimetres off is fine.

| # | Item | Where | Resting pose | x [cm] | y [cm] | yaw [deg] |
|---|---|---|---|---|---|---|
| 1 | blue-block | table | flat | 32.7 | 9.8 | -36 |
| 2 | blue-cup-with-yellow-stripe | table | upright | 67.4 | 20.2 | -155 |
| 3 | green-block | table | flat | 12.7 | 22.1 | 166 |
| 4 | green-mug | table | upright | 26.6 | 15.9 | 163 |
| 5 | orange-block | table | flat | 41.3 | -23.2 | 138 |
| 6 | red-block | table (picked up before the plan: it starts in the gripper) | flat | 17.7 | -22.0 | 75 |
| 7 | red-can | table | upright | 27.2 | -2.6 | -118 |
| 8 | red-spray-bottle | table | upright | 26.3 | -11.0 | 178 |
| 9 | roll-of-black-tape | basket | flat | 56.1 | -6.6 | 148 |
| 10 | roll-of-blue-tape | table | flat | 39.3 | 21.0 | 35 |
| 11 | roll-of-green-tape | table | flat | 31.8 | -22.3 | 5 |
| 12 | yellow-block | table | flat | 51.1 | 22.5 | 61 |
