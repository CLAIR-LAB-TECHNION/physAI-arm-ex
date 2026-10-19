# Verification of the autonomous actions (simulation)

Every action was executed by the skill server from the robot's own perception. Afterwards the
simulator's ground truth, which the robot never sees, was checked: the PDDL facts must be exactly
the action's effect, and no other item may have moved more than 2 cm or tilted more than 10 deg.
A *refusal* is the robot declining an action it cannot do safely (no free spot, no collision-free
grasp, item not found): nothing moves and the state is unchanged.

Runs: `main` = 6 problems, actions chosen to cover every item (verify_report.json); `basket` = tall items into an empty basket (verify_basket.json); `basket-2` = the same after the far-wall preference (verify_basket2.json); `video` = the ten videos (`../videos/*.json`), recorded at 0.3 x real time.

## 2026-10-14: the consoles, and the depth obstacle model

**The four `run/` scripts** were run in simulation as a user would, with the consoles driven
through a terminal (pty) and the result read from Gazebo's ground truth afterwards:

* Action console: `hello`, `(pick-up gren-mug)`, `(pick-up)`, `(fly green-mug)` answered with
  `invalid action string: ...`; `(put-on-table red-can)` with an empty gripper and
  `(pick-up blue-block)` while holding a block answered with `action failed: not applicable: ...`;
  `(pick-up green-mug)`, `(put-in-basket green-mug)`, `(pick-up red-block)`,
  `(put-on-table red-block)` succeeded, and the ground truth agreed (mug in the basket, the rest
  on the table, gripper empty). Ctrl-C during a pick-up stopped the arm (the controller cancelled
  the goal) and the prompt came back; Ctrl-C at the prompt quit.
* Goal console, from all twelve items on the table: a misspelt item and a missing `)` answered
  with `invalid goal: ...`; `(on-table red-can)` with `the goal already holds`;
  `(in-basket red-can) (robot-gripping yellow-block)` planned and performed (3 actions);
  `(on-table red-can)` then answered with `the goal cannot be reached from this state`;
  `(or (in-basket blue-block) (in-basket red-block))` planned and performed (3 actions). Ground
  truth: red-can, yellow-block and blue-block in the basket. Then
  `(in-basket red-spray-bottle) (in-basket green-mug) (robot-gripping red-can)` (5 actions): the
  first four succeeded; the last was refused because the bottle had landed leaning across the
  basket with its head over the rim towards the robot's base, where the arm would have passed.
* Real-robot mode with the UR driver's mock hardware and a fake OnRobot gripper: both consoles
  connect to the arm and the gripper, load SAM 3 and, with no RealSense plugged in, stop with
  `no camera images on ... Is the camera running?`.

**Fixes found on the way** (`depth_model_v2/`, the same scenes as `depth_model/`, 8 actions each):

* The upright spray bottle could not be picked: lifting it, its box touched unseen cells inside
  its own outline that had been given to a neighbouring item. Now every cell inside an item's
  fitted outline (grown 1 cm) is the item's own, unless another item's mask covers it.
* Holding the spray bottle, no put was possible: at the tilted survey view the bottle swings out
  over a strip the camera does not see, and the way back to a level gripper was blocked. Now the
  space the held item filled when a frame was taken bounds that unseen space, and with an item in
  the gripper the views are raised (up to 6 cm) until it hangs above the unseen-space height.
* The consoles (and the skill server) could hang at start-up when the controller manager dropped
  a reply; the request is now repeated.

| run | pick-up | put-in-basket | put-on-table |
|---|---|---|---|
| `depth_model` (2026-10-13) | 13/24 verified, 10 refused, 1 failed | 6/10 verified, 4 refused | 5/6 verified, 1 refused |
| `depth_model_v2` (2026-10-14) | 18/23 verified, 4 refused, 1 failed | 9/9 verified | 8/8 verified |

All spray-bottle refusals of 2026-10-13 are gone. The remaining refusals are all in problem 710:
items touching the basket from outside (the open finger would hit the wall), and once no way
home that keeps the arm out of the basket. The one failure (in both runs) is a refusal during
which the bottle standing in the basket of problem 710 slid by 2 cm without tilting, as the scene
settled. Problem 706 did not start in the first run (the simulator's controller manager timed
out) and was run again on its own: `verify_706.log`, `verify_report_706.json`.

## Revision of 2026-10-08 afternoon: drops into the basket, top-view survey, 16-sided round items

Changes: put-in-basket drops the item from above the rim over the emptiest spot (never refused for lack of room);
the survey is the top view only, plus the front view before a pick-up whose target the top view did not
recognise; round items collide as 16-sided prisms (the octagons let the fingers close on corners, leaving a
visible gap). A drop may move items already in the basket: that is listed but not a failure, unless one
is knocked out of the basket. Runs: `drop-seq` = problem 702, a fixed sequence filling the basket with the
tall items (verify_drop_sequence.json); `drop` = problems 704, 705, 708, actions chosen by the tool, seed 3
(verify_drop.json).

| action | executed | verified | refused (nothing moved) | failed | items verified |
|---|---|---|---|---|---|
| pick-up | 25 | 17 | 7 | 1 | 12 |
| put-in-basket | 10 | 10 | 0 | 0 | 8 |
| put-on-table | 6 | 6 | 0 | 0 | 6 |

Refusals: `red-spray-bottle not found` (7): in problems 705 and 708 the bottle stands with its nozzle
towards the front view and SAM 3 does not name it from either view (the earlier runs show the same: it was
never picked there); `(pick-up blue-block)` in 705: the block lies against the basket and the open finger
would hit the wall; `(pick-up yellow-block)` at the end of drop-seq: the spray bottle, dropped upright onto a
pile 12 cm high, toppled and sticks out over the rim, and as an unknown obstacle (a box from the table up)
it blocks the arm's way. Mean time per action 33 s (was 64 s).

The ten videos (`../videos/*.json`) were recorded again with this version, the same problems and
actions as before: all ten verified; video length 18-32 s each (was 37-52 s). The previous recordings
are in `../videos/previous/`.

| run | problem | action | result | time | robot's message; other items changed |
|---|---|---|---|---|---|
| drop-seq | 702 | `(pick-up roll-of-green-tape)` | verified | 29 s | holding roll-of-green-tape (width 82 mm) |
| drop-seq | 702 | `(put-in-basket roll-of-green-tape)` | verified | 17 s | put roll-of-green-tape at (0.353, 0.052) on the basket |
| drop-seq | 702 | `(pick-up red-can)` | verified | 36 s | holding red-can (width 59 mm) |
| drop-seq | 702 | `(put-in-basket red-can)` | verified | 18 s | put red-can at (0.411, 0.030) on the basket; in the basket: roll-of-green-tape moved 3.8 cm, tilt 0 deg |
| drop-seq | 702 | `(pick-up green-mug)` | verified | 41 s | holding green-mug (width 65 mm) |
| drop-seq | 702 | `(put-in-basket green-mug)` | verified | 39 s | put green-mug at (0.352, 0.080) on the basket |
| drop-seq | 702 | `(pick-up red-spray-bottle)` | verified | 46 s | holding red-spray-bottle (width 34 mm) |
| drop-seq | 702 | `(put-in-basket red-spray-bottle)` | verified | 41 s | put red-spray-bottle at (0.322, 0.060) on the basket |
| drop-seq | 702 | `(pick-up roll-of-blue-tape)` | verified | 38 s | holding roll-of-blue-tape (width 86 mm) |
| drop-seq | 702 | `(put-on-table roll-of-blue-tape)` | verified | 30 s | put roll-of-blue-tape at (0.530, 0.220) on the table |
| drop-seq | 702 | `(pick-up yellow-block)` | FAILED | 27 s | home: arm hits unknown obstacle 2; red-spray-bottle moved 1.8 cm, tilt 18 deg; green-mug moved 2.1 cm, tilt 30 deg |
| drop | 704 | `(pick-up green-mug)` | verified | 45 s | holding green-mug (width 65 mm) |
| drop | 704 | `(put-in-basket green-mug)` | verified | 41 s | put green-mug at (0.321, 0.029) on the basket |
| drop | 704 | `(pick-up yellow-block)` | verified | 49 s | holding yellow-block (width 50 mm) |
| drop | 704 | `(put-on-table yellow-block)` | verified | 36 s | put yellow-block at (0.580, 0.050) on the table |
| drop | 704 | `(pick-up roll-of-blue-tape)` | verified | 40 s | holding roll-of-blue-tape (width 85 mm) |
| drop | 704 | `(put-in-basket roll-of-blue-tape)` | verified | 34 s | put roll-of-blue-tape at (0.320, 0.089) on the basket |
| drop | 704 | `(pick-up green-block)` | verified | 43 s | holding green-block (width 56 mm) |
| drop | 704 | `(put-on-table green-block)` | verified | 41 s | put green-block at (0.690, -0.090) on the table |
| drop | 704 | `(pick-up orange-block)` | verified | 43 s | holding orange-block (width 46 mm) |
| drop | 704 | `(put-in-basket orange-block)` | verified | 39 s | put orange-block at (0.400, 0.009) on the basket |
| drop | 705 | `(pick-up red-spray-bottle)` | refused | 20 s | red-spray-bottle not found |
| drop | 705 | `(pick-up red-can)` | verified | 36 s | holding red-can (width 58 mm) |
| drop | 705 | `(put-on-table red-can)` | verified | 30 s | put red-can at (0.520, 0.200) on the table |
| drop | 705 | `(pick-up roll-of-black-tape)` | verified | 31 s | holding roll-of-black-tape (width 87 mm) |
| drop | 705 | `(put-in-basket roll-of-black-tape)` | verified | 21 s | put roll-of-black-tape at (0.301, 0.030) on the basket |
| drop | 705 | `(pick-up blue-block)` | refused | 20 s | cannot pick blue-block: descend: gripper or object hits the basket wall |
| drop | 705 | `(pick-up red-spray-bottle)` | refused | 25 s | red-spray-bottle not found |
| drop | 705 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 39 s | holding blue-cup-with-yellow-stripe (width 67 mm) |
| drop | 705 | `(put-on-table blue-cup-with-yellow-stripe)` | verified | 29 s | put blue-cup-with-yellow-stripe at (0.170, 0.210) on the table |
| drop | 705 | `(pick-up red-block)` | verified | 40 s | holding red-block (width 60 mm) |
| drop | 708 | `(pick-up red-spray-bottle)` | refused | 21 s | red-spray-bottle not found |
| drop | 708 | `(pick-up blue-block)` | verified | 43 s | holding blue-block (width 42 mm) |
| drop | 708 | `(put-in-basket blue-block)` | verified | 26 s | put blue-block at (0.381, 0.010) on the basket |
| drop | 708 | `(pick-up red-spray-bottle)` | refused | 20 s | red-spray-bottle not found |
| drop | 708 | `(pick-up roll-of-green-tape)` | verified | 39 s | holding roll-of-green-tape (width 83 mm) |
| drop | 708 | `(put-on-table roll-of-green-tape)` | verified | 25 s | put roll-of-green-tape at (0.160, 0.200) on the table |
| drop | 708 | `(pick-up red-spray-bottle)` | refused | 23 s | red-spray-bottle not found |
| drop | 708 | `(pick-up roll-of-blue-tape)` | verified | 39 s | holding roll-of-blue-tape (width 86 mm) |
| drop | 708 | `(put-in-basket roll-of-blue-tape)` | verified | 24 s | put roll-of-blue-tape at (0.381, 0.010) on the basket |
| drop | 708 | `(pick-up red-spray-bottle)` | refused | 21 s | red-spray-bottle not found |

## Earlier runs (before the revision)

### Summary

| action | executed | verified | refused (nothing moved) | failed | items verified |
|---|---|---|---|---|---|
| pick-up | 40 | 37 | 1 | 2 | 12 of 12 |
| put-in-basket | 25 | 21 | 4 | 0 | 12 of 12 |
| put-on-table | 13 | 13 | 0 | 0 | 10 of 12 |

Both failures come from one bug in the *simulated* gripper driver (problem 702: under load the
fingers took over 10 s to settle and the driver then did not set its grip flag; the robot believed
the yellow block had slipped, and dropped it when it opened for the next pick; the later "yellow-block
not found" is a correct refusal, the block had fallen out of view). Fixed in
lab_replica/robot/rg6_driver.py; nothing failed in the 56 actions run after the fix. The real RG6
reports its grip flag itself.

Not in the table: one more refusal while recording the videos (problem 710, `(pick-up red-block)`: no
collision-free path in a tight layout, see the main README, A.7); that video was recorded in a new
problem (716) instead.

### Every action

| run | problem | action | result | time | robot's message; other items changed |
|---|---|---|---|---|---|
| main | 700 | `(pick-up red-spray-bottle)` | verified | 56 s | holding red-spray-bottle (width 34 mm) |
| main | 700 | `(put-in-basket red-spray-bottle)` | refused | 45 s | no placement for red-spray-bottle on the basket: no free spot |
| main | 700 | `(put-on-table red-spray-bottle)` | verified | 60 s | put red-spray-bottle at (0.530, 0.190) on the table |
| main | 700 | `(pick-up roll-of-black-tape)` | verified | 54 s | holding roll-of-black-tape (width 85 mm) |
| main | 700 | `(put-in-basket roll-of-black-tape)` | verified | 51 s | put roll-of-black-tape at (0.392, 0.071) on the basket |
| main | 700 | `(pick-up blue-block)` | verified | 58 s | holding blue-block (width 42 mm) |
| main | 700 | `(put-in-basket blue-block)` | verified | 63 s | put blue-block at (0.382, -0.009) on the basket |
| main | 700 | `(pick-up red-can)` | verified | 56 s | holding red-can (width 60 mm) |
| main | 701 | `(pick-up roll-of-blue-tape)` | verified | 67 s | holding roll-of-blue-tape (width 92 mm) |
| main | 701 | `(put-on-table roll-of-blue-tape)` | verified | 49 s | put roll-of-blue-tape at (0.620, -0.190) on the table |
| main | 701 | `(pick-up red-block)` | verified | 69 s | holding red-block (width 60 mm) |
| main | 701 | `(put-in-basket red-block)` | verified | 90 s | put red-block at (0.352, 0.081) on the basket |
| main | 701 | `(pick-up roll-of-green-tape)` | verified | 78 s | holding roll-of-green-tape (width 82 mm) |
| main | 701 | `(put-on-table roll-of-green-tape)` | verified | 64 s | put roll-of-green-tape at (0.580, -0.030) on the table |
| main | 701 | `(pick-up orange-block)` | verified | 94 s | holding orange-block (width 46 mm) |
| main | 701 | `(put-on-table orange-block)` | verified | 96 s | put orange-block at (0.580, 0.100) on the table |
| main | 702 | `(pick-up yellow-block)` | FAILED | 64 s | yellow-block slipped out of the gripper |
| main | 702 | `(pick-up green-block)` | FAILED | 77 s | holding green-block (width 56 mm); yellow-block moved 42.4 cm, tilt 90 deg |
| main | 702 | `(put-in-basket green-block)` | verified | 56 s | put green-block at (0.352, 0.051) on the basket |
| main | 702 | `(pick-up yellow-block)` | refused | 30 s | yellow-block not found |
| main | 702 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 50 s | holding blue-cup-with-yellow-stripe (width 71 mm) |
| main | 702 | `(put-in-basket blue-cup-with-yellow-stripe)` | refused | 39 s | no placement for blue-cup-with-yellow-stripe on the basket: no free spot |
| main | 702 | `(put-on-table blue-cup-with-yellow-stripe)` | verified | 49 s | put blue-cup-with-yellow-stripe at (0.230, -0.150) on the table |
| main | 702 | `(pick-up green-mug)` | verified | 77 s | holding green-mug (width 65 mm) |
| main | 703 | `(pick-up yellow-block)` | verified | 63 s | holding yellow-block (width 50 mm) |
| main | 703 | `(put-in-basket yellow-block)` | verified | 76 s | put yellow-block at (0.362, 0.111) on the basket |
| main | 703 | `(pick-up green-block)` | verified | 65 s | holding green-block (width 56 mm) |
| main | 703 | `(put-in-basket green-block)` | verified | 78 s | put green-block at (0.302, 0.081) on the basket |
| main | 703 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 50 s | holding blue-cup-with-yellow-stripe (width 71 mm) |
| main | 703 | `(put-on-table blue-cup-with-yellow-stripe)` | verified | 54 s | put blue-cup-with-yellow-stripe at (0.630, 0.100) on the table |
| main | 703 | `(pick-up red-spray-bottle)` | verified | 53 s | holding red-spray-bottle (width 34 mm) |
| main | 703 | `(put-on-table red-spray-bottle)` | verified | 78 s | put red-spray-bottle at (0.580, -0.010) on the table |
| main | 704 | `(pick-up orange-block)` | verified | 65 s | holding orange-block (width 46 mm) |
| main | 704 | `(put-in-basket orange-block)` | verified | 83 s | put orange-block at (0.292, 0.081) on the basket |
| main | 704 | `(pick-up red-block)` | verified | 60 s | holding red-block (width 60 mm) |
| main | 704 | `(put-in-basket red-block)` | verified | 82 s | put red-block at (0.312, 0.011) on the basket |
| main | 704 | `(pick-up blue-block)` | verified | 69 s | holding blue-block (width 42 mm) |
| main | 704 | `(put-on-table blue-block)` | verified | 71 s | put blue-block at (0.190, -0.110) on the table |
| main | 704 | `(pick-up green-mug)` | verified | 72 s | holding green-mug (width 65 mm) |
| main | 704 | `(put-on-table green-mug)` | verified | 105 s | put green-mug at (0.550, -0.190) on the table |
| main | 705 | `(pick-up roll-of-blue-tape)` | verified | 59 s | holding roll-of-blue-tape (width 92 mm) |
| main | 705 | `(put-in-basket roll-of-blue-tape)` | verified | 55 s | put roll-of-blue-tape at (0.312, 0.081) on the basket |
| main | 705 | `(pick-up roll-of-black-tape)` | verified | 53 s | holding roll-of-black-tape (width 90 mm) |
| main | 705 | `(put-in-basket roll-of-black-tape)` | verified | 47 s | put roll-of-black-tape at (0.312, 0.081) on the basket |
| main | 705 | `(pick-up red-can)` | verified | 75 s | holding red-can (width 59 mm) |
| main | 705 | `(put-on-table red-can)` | verified | 68 s | put red-can at (0.510, 0.190) on the table |
| main | 705 | `(pick-up yellow-block)` | verified | 88 s | holding yellow-block (width 50 mm) |
| main | 705 | `(put-on-table yellow-block)` | verified | 102 s | put yellow-block at (0.690, -0.030) on the table |
| basket | 720 | `(pick-up green-mug)` | verified | 58 s | holding green-mug (width 65 mm) |
| basket | 720 | `(put-in-basket green-mug)` | verified | 92 s | put green-mug at (0.352, 0.051) on the basket |
| basket | 720 | `(pick-up roll-of-green-tape)` | verified | 57 s | holding roll-of-green-tape (width 82 mm) |
| basket | 720 | `(put-in-basket roll-of-green-tape)` | verified | 45 s | put roll-of-green-tape at (0.322, 0.091) on the basket |
| basket | 721 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 55 s | holding blue-cup-with-yellow-stripe (width 72 mm) |
| basket | 721 | `(put-in-basket blue-cup-with-yellow-stripe)` | verified | 54 s | put blue-cup-with-yellow-stripe at (0.352, 0.051) on the basket |
| basket | 721 | `(pick-up red-can)` | verified | 53 s | holding red-can (width 63 mm) |
| basket | 721 | `(put-in-basket red-can)` | refused | 30 s | no placement for red-can on the basket: no free spot |
| basket | 722 | `(pick-up red-spray-bottle)` | verified | 54 s | holding red-spray-bottle (width 34 mm) |
| basket | 722 | `(put-in-basket red-spray-bottle)` | verified | 66 s | put red-spray-bottle at (0.352, 0.051) on the basket |
| basket | 722 | `(pick-up red-can)` | verified | 58 s | holding red-can (width 62 mm) |
| basket | 722 | `(put-in-basket red-can)` | refused | 37 s | no placement for red-can on the basket: no free spot |
| basket-2 | 721 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 56 s | holding blue-cup-with-yellow-stripe (width 72 mm) |
| basket-2 | 721 | `(put-in-basket blue-cup-with-yellow-stripe)` | verified | 52 s | put blue-cup-with-yellow-stripe at (0.402, 0.061) on the basket |
| basket-2 | 721 | `(pick-up red-can)` | verified | 50 s | holding red-can (width 63 mm) |
| basket-2 | 721 | `(put-in-basket red-can)` | verified | 43 s | put red-can at (0.292, 0.021) on the basket |
| basket-2 | 720 | `(pick-up green-mug)` | verified | 57 s | holding green-mug (width 65 mm) |
| basket-2 | 720 | `(put-in-basket green-mug)` | verified | 95 s | put green-mug at (0.382, 0.051) on the basket |
| basket-2 | 720 | `(pick-up red-can)` | verified | 57 s | holding red-can (width 59 mm) |
| basket-2 | 720 | `(put-in-basket red-can)` | verified | 57 s | put red-can at (0.292, 0.071) on the basket |
| video | 706 | `(pick-up green-mug)` | verified | 156 s* | holding green-mug (width 65 mm) |
| video | 707 | `(pick-up red-spray-bottle)` | verified | 144 s* | holding red-spray-bottle (width 34 mm) |
| video | 708 | `(pick-up blue-cup-with-yellow-stripe)` | verified | 152 s* | holding blue-cup-with-yellow-stripe (width 67 mm) |
| video | 709 | `(put-on-table roll-of-blue-tape)` | verified | 139 s* | put roll-of-blue-tape at (0.620, 0.010) on the table |
| video | 716 | `(put-in-basket red-block)` | verified | 121 s* | put red-block at (0.342, 0.091) on the basket |
| video | 711 | `(pick-up red-can)` | verified | 169 s* | holding red-can (width 60 mm) |
| video | 712 | `(put-in-basket roll-of-green-tape)` | verified | 129 s* | put roll-of-green-tape at (0.302, 0.041) on the basket |
| video | 713 | `(put-in-basket yellow-block)` | verified | 128 s* | put yellow-block at (0.402, 0.071) on the basket |
| video | 714 | `(pick-up roll-of-black-tape)` | verified | 153 s* | holding roll-of-black-tape (width 92 mm) |
| video | 715 | `(put-on-table green-block)` | verified | 131 s* | put green-block at (0.210, -0.190) on the table |

\* wall time at 0.3 x real time.
