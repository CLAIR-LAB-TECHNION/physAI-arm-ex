# physAI-arm-ex

During the week you saw the demo. A **UR5e** robot arm with an **OnRobot
RG6** gripper and a **RealSense** depth camera on its wrist assembled an
order. Someone typed a goal such as `(in-basket red-block)`, and the robot
planned the actions, looked at the table, recognised the items, grasped them
and put them where the goal wanted them.

In this exercise you implement the key components of that demo. The robot's
drivers, the camera, the segmentation model (SAM 3), the motion planner
(MoveIt) and the consoles you type into are given. What is missing is what
the robot decides.

* **Task planning** turns a goal into a sequence of actions (PDDL, the
  Unified Planning Framework, Fast Downward).
* **State estimation** turns camera images into where the items are and how
  to grasp them.
* **Motion planning** turns a grasp or a place into collision-free arm
  motions (MoveIt).
* **Skills** combine them into the three actions the robot performs,
  `(pick-up X)`, `(put-on-table X)` and `(put-in-basket X)`.

All of it is plain Python, without ROS, in
[`src/lab_replica/lab_replica/exercise/`](src/lab_replica/lab_replica/exercise/).
Every public function and constant there is yours to write. Each is marked
with `# TODO`, its docstring says what it must return and gives hints, and
until you write it, it raises `NotImplementedError` with its name. The
building blocks you may use are in [the toolkit](#the-toolkit).

**Contents**

1. [Set up (once)](#set-up-once)
2. [How to work](#how-to-work)
3. [Warm-up. The observation configuration](#warm-up-the-observation-configuration)
4. [Task 1. Task planning](#task-1-task-planning)
5. [Task 2. Seeing blocks](#task-2-seeing-blocks)
6. [Task 3. Moving and looking](#task-3-moving-and-looking)
7. [Task 4. Picking up a block](#task-4-picking-up-a-block)
8. [Task 5. Putting a block down](#task-5-putting-a-block-down)
9. [Task 6. Every item](#task-6-every-item)
10. [Task 7. New items](#task-7-new-items)
11. [Bonus. Improve the framework](#bonus-improve-the-framework)
12. [The scripts](#the-scripts)
13. [The toolkit](#the-toolkit)

---

## Set up (once)

The lab PCs run Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/)
and an NVIDIA GPU, and the system packages are installed. In any directory

```console
git clone https://github.com/guyazran/physAI-arm-ex.git
cd physAI-arm-ex
run/setup.sh
```

`run/setup.sh` makes a Python environment (`.venv`) with SAM 3 and the
planner, downloads SAM 3, builds the workspace and checks that everything
works. It needs no `sudo`. If system packages are missing, it prints the
`apt` command for an administrator and stops.

* SAM 3 ([`facebook/sam3`](https://huggingface.co/facebook/sam3)) is a gated
  model. Before running the script, accept its licence on Hugging Face and
  make a read token (Settings, Access Tokens). The script asks for it.
* The commands in this README run in the clone's top directory (the one
  with `run/` and `src/`).
* After `git pull`, run `run/setup.sh` again. Edits to the exercise files
  need no rebuild.

## How to work

**The robot.** The UR driver, the RG6 gripper driver and the camera run on
other lab computers, started by the staff ([docs/lab_setup.md](docs/lab_setup.md)).
Your PC runs one console at a time, which talks to them over ROS 2 on the
lab's `ROS_DOMAIN_ID` (131, printed by every script). Before the robot moves,
press **Play** on the teach pendant. Keep a hand on the emergency stop.
Ctrl-C in the console stops the arm.

**Try it on the robot,** slowly at first.

```console
run/actions.sh real --speed-scale 0.3       # one action at a time, e.g. (pick-up red-block)
run/goals.sh real --speed-scale 0.3 --step  # a goal, e.g. (in-basket red-block)
```

The consoles open a camera window that shows every view the robot takes,
annotated with what your code found and decided. The run directory (printed
when the console is ready) keeps these pictures and a log of every step.
A function you have not written yet makes the action fail with its name.

## Warm-up. The observation configuration

Every action starts by looking at the whole workspace from one arm
configuration, `OBSERVATION_JOINTS` in
[`exercise/state_estimation.py`](src/lab_replica/lab_replica/exercise/state_estimation.py),
the six joint angles in radians from `shoulder_pan` to `wrist_3`. From there
the wrist camera must see the whole blue board, with the basket anywhere on
it, close enough for SAM 3 to recognise every item. The arm must stand there
without touching the table or the wall, also when it holds a tall item.

Find it on the robot.

```console
run/workspace_view.sh real
```

A window shows the camera live with SAM 3's masks of the items and the
basket, and lists the items it does not find. The terminal prints the arm's
joints as a line to copy, updating as the arm moves. The workspace view never
moves the arm. Put a few blocks and the basket on the board, put the pendant
in freedrive and move the arm by hand until every item is found, wherever it
stands on the board. Then copy the `OBSERVATION_JOINTS = [...]` line into the
file. In the window, `s` keeps a line to compare candidates and `q` quits.

*Hints.* Farther away sees more, but smaller. Move the basket to the board's
corners and check that it is still found. Watch for the gripper's fingers at
the edge of the image.

## Task 1. Task planning

Files [`exercise/task_planning/`](src/lab_replica/lab_replica/exercise/task_planning/).

| What | Where |
|---|---|
| the preconditions and effects of `pick-up`, `put-on-table` and `put-in-basket` (their names, parameters and the four predicates stay as they are) | `domain.pddl` |
| the objects (start with the five blocks) and a goal to test with | `problem.pddl` |
| `load_problem`, `solve`, `facts_of`, `expected_states`, `execute_plan` with the [Unified Planning Framework](https://unified-planning.readthedocs.io/en/latest/) and its Fast Downward engine | `planning.py` |

The goal console reads `problem.pddl`, keeps its objects and puts in the
current state and the goal you type. To try your planner, start the goal
console with `--step`. It prints the plan and waits for Enter before every
action, and `n` stops. Performing the plan needs the skills of tasks 3 to 5.

*Hints.* The robot holds at most one item, and nothing ever takes an item
out of the basket. An effect that makes a fact true must also make the facts
it contradicts false. UPF's documentation has short examples of reading
PDDL, solving a problem and simulating a plan, linked from `planning.py`.

## Task 2. Seeing blocks

File [`exercise/state_estimation.py`](src/lab_replica/lab_replica/exercise/state_estimation.py).
The robot calls these after every look.

| Function | What it does |
|---|---|
| `pixels_to_world` | depth pixels to 3D points in the world |
| `estimate_placement` | an item's points to where and how it rests, and how plausible the points are for that item |
| `assign_masks` | SAM 3's candidate masks in one image to at most one per item |
| `fuse_views` | the items found in several images to one detection per item |

For now, only blocks. You will see what your code finds in the camera window
in task 3.

*Hints.* `pixels_to_world` runs on every pixel of every image, so use numpy
arrays rather than loops. A block rests flat, and its top seen from above is
a square. SAM 3 sometimes puts a wrong name on a mask, so a plausibility
that compares the top's height and the outline's size with the model's
keeps the wrong names out.

## Task 3. Moving and looking

File [`exercise/motion_planning.py`](src/lab_replica/lab_replica/exercise/motion_planning.py).
The robot moves to its camera views and back home with these.

| Function | What it does |
|---|---|
| `choose_solution` | of the inverse kinematics solutions, the one nearest to the arm's configuration |
| `camera_to_gripper`, `lifted`, `turned` | small pose helpers |
| `joint_line`, `straight_path`, `plan_path` | the simplest collision-free path between two configurations |
| `move_to_pose` | a collision-free path to a gripper pose |
| `plan_observation`, `plan_view`, `plan_home` | the steps to a configuration to look from, to a camera pose, and home |
| `held_box` | the held item's box in the gripper's frame, which MoveIt carries with the gripper |
| `scene_obstacles` | what MoveIt keeps clear of besides the table and the wall |

And the constants `JOINT_WEIGHTS`, `RISE_Z`, `ITEM_PAD` and `BASKET_PAD`.

To try them, write a first `pick_up` in
[`exercise/skills.py`](src/lab_replica/lab_replica/exercise/skills.py) that
only looks (`robot.survey(se.OBSERVATION_JOINTS)`) and returns a message,
and, with a few blocks and the basket on the board, type
`(pick-up red-block)` in the action console. The arm moves to the
observation configuration, the camera window shows the blocks your task 2
code found and the obstacles, and the arm returns home.

*Hints.* MoveIt's planner always finds a way if there is one, but often a
long detour. Above the table a straight line in joint space is usually free.
An obstacle slightly too large costs little, one too small costs a collision.

## Task 4. Picking up a block

| Function | File | What it does |
|---|---|---|
| `grasp_in_image` | `state_estimation.py` | grasps proposed in the image, each a pixel and a closing direction |
| `grasp_pose` | `state_estimation.py` | a grasp in the image to the gripper's 3D pose and opening |
| `plan_pick` | `motion_planning.py` | above the grasp, open, straight down, close, straight up |
| `pick_up` | `skills.py` | the action `(pick-up X)` |

And the constants `GRASP_BAND`, `TIP_CLEARANCE`, `KNUCKLE_MARGIN`,
`OPEN_MARGIN`, `SQUEEZE`, `APPROACH` and `SLOW`. Try
`(pick-up red-block)` in the action console. The camera window shows the
grasps you proposed and the one planned.

*Hints.* The fingers close on the top of the block, across a pair of
opposite faces. The RG6's knuckles reach between the pads about 6 cm above
the fingertips, so grasp deep, but never touch the table. `pick_up` must tell
the robot how it holds the item (`robot.now_holding`), because the put
actions start from there.

## Task 5. Putting a block down

| Function | File | What it does |
|---|---|---|
| `gripper_for` | `motion_planning.py` | the gripper's pose that puts the held item at a given place |
| `plan_place` | `motion_planning.py` | carry, straight down, open, straight up |
| `sample_spots`, `is_free`, `find_table_spot` | `motion_planning.py` | a free spot on the table that the robot can reach |
| `put_on_table`, `put_in_basket` | `skills.py` | the actions `(put-on-table X)` and `(put-in-basket X)` |

And the constants `PLACE_APPROACH`, `DROP_APPROACH`, `RELEASE_GAP` and
`DROP_CLEARANCE`. Try in the action console `(pick-up red-block)`, `(put-in-basket red-block)`,
`(pick-up blue-block)` and `(put-on-table blue-block)`.

**Now the demo works with blocks.** Put every block on the board, the basket
empty, start the goal console and type
`(in-basket red-block) (in-basket blue-block)`.

*Hints.* In the basket, items land on each other. `robot.drop_spots` says how
high the pile is at each spot. The best spot may be out of reach, so keep
a few to try.

## Task 6. Every item

Now the other items of
[`config/items.yaml`](src/lab_replica/config/items.yaml), a can, a cup, a
mug, tape rolls and a spray bottle. Add them to `problem.pddl`.

| What | Where |
|---|---|
| `estimate_placement` for every kind of item, by matching its model seen from above to the points (`height_map`, `pose_cost`, `fit_pose` and the constants `MAP_CELL`, `TRUNCATE`, `BELOW_WEIGHT`, `SEARCH`) | `state_estimation.py` |
| `grasp_in_image` for elongated and round tops (`ROUND_ASPECT`, `SQUARE_FILL`) | `state_estimation.py` |
| more views when the top view is not enough (`SIDE_VIEW`, `CLOSE_HEIGHT`, `close_views`, and `RECOGNISED` in `pick_up`) | `state_estimation.py`, `skills.py` |

Try picking up each item in the action console.

*Hints.* `toolkit.model_height_map` gives the top surface of any item seen
from above, made from its collision shapes. Matching heights rather than
outlines lets the same code work for every kind of item. Use the points
around the mask too, since where the camera sees the table, the item is not.
From straight above SAM 3 does not recognise every item, and a closer view
gives a more accurate pose.

## Task 7. New items

File [`exercise/misc.py`](src/lab_replica/lab_replica/exercise/misc.py).
`new_item` and `forget_item` are behind the console commands

```console
new-item: purple-block                          # look for "purple block", measure it, add it
new-item: lid prompt="round white lid" shape=round
forget-item: purple-block
```

The robot finds the item by a text prompt (SAM 3), measures it with the
depth camera and knows it until the console quits. In the goal console it
also becomes an object of the planning problem. Start with boxes
(`shape=box`), then round items. Test with an item that is not one of the
twelve.

*Hints.* `toolkit.measure_item` measures an item from its points. Erosion
makes the mask, and so the item, smaller. Refuse what the RG6 cannot grasp.

## Bonus. Improve the framework

Everything in the repository may be changed. Some ideas follow.

* **Stacking.** A predicate `(on ?a ?b)` and actions that put an item on
  another and take it off, a place on top of another item's top surface, and
  a state estimation that tells an item on another from one on the table.
* **Mapping the 3D environment.** The obstacles MoveIt avoids are boxes
  around what the camera saw. Fuse the depth of every view into a 3D map (a
  voxel grid, or MoveIt's [Octomap](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html))
  for closer, safer motions around unknown objects.
* **Integrating the task and motion domains.** When an item cannot be
  grasped because its neighbours are in the way, move them out of the way
  first. The task planner then needs to know about geometry, for example a
  predicate `(blocks ?a ?b)` from the state estimation, or a new plan after a
  motion fails.
* **Closing the loop.** Look after every action to check its effect, instead
  of assuming it, and plan again when the world is not what the plan
  expected.

---

## The scripts

| Script | What it does | Use it for |
|---|---|---|
| `run/setup.sh` | sets up the Python environment, SAM 3 and the build (`--no-model` skips SAM 3, `--fresh-venv` starts over) | [setting up](#set-up-once), and after `git pull` |
| `run/workspace_view.sh real` | the camera live with SAM 3's masks, and the arm's joints as they change | the [warm-up](#warm-up-the-observation-configuration) |
| `run/actions.sh real` | the [action console](#the-action-console), one action at a time | tasks 3 to 7 |
| `run/goals.sh real` | the [goal console](#the-goal-console), a goal planned and performed | tasks 5 to 7 |
| `src/lab_replica/tools/depth_map.py` | runs the perception (with your state estimation) on saved camera frames, without the robot | debugging state estimation |
| `run/sim.sh` | the Gazebo simulation of the cell (needs extra packages, see [docs/lab_setup.md](docs/lab_setup.md)) | the staff |

The `run/` scripts set up ROS 2, the workspace and the `.venv` themselves,
and work from any directory. Run only one console at a time, since each
drives the robot. `--help` after a script lists its options. The Python tools
need ROS 2, the workspace and the `.venv` first.

```console
source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
python3 src/lab_replica/tools/depth_map.py runs/<run>/frame_*.npz --pick red-block
```

### The action console

```text
$ run/actions.sh real --speed-scale 0.3
Ready. Log and camera views: /home/student/physAI-arm-ex/runs/20261022_101500
action> (pick-up red-block)
performing action (pick-up red-block)
action success: holding red-block (width 49 mm)
action> (put-in-basket red-block)
```

An action is its name and one item, as in `(pick-up red-block)`. The answer
is `action success` with your skill's message, or `action failed` with the
error. Tab completes names, and the arrow keys bring back earlier lines.
`new-item:` and `forget-item:` add and remove [new items](#task-7-new-items).
Ctrl-D quits.

| Option | What it does |
|---|---|
| `--speed-scale S` | slower (below 1) or faster motions |
| `--no-window` | no camera window |
| `--no-home` | stay where the action ended instead of returning home |
| `--verbose` | print every step, not only the current phase |
| `--save-frames` | also save every camera image (for `depth_map.py`) |

### The goal console

```text
$ run/goals.sh real --speed-scale 0.3 --step
goal> (in-basket red-block) (robot-gripping blue-block)
planning ...
plan (3 actions):
  1. (pick-up red-block)
  2. (put-in-basket red-block)
  3. (pick-up blue-block)
```

A goal is one or more facts that must all hold at the end, over the four
predicates of the domain, combined as in PDDL with `and`, `or`, `not`,
`imply`, `forall` and `exists`, for example
`(forall (?x - item) (in-basket ?x))`. The console plans from the current
state with your planner and performs the plan with your skills, stopping
at the first failure.

The console starts from every object of `problem.pddl` on the table and the
gripper empty. After each successful action it updates the state with the
action's expected effect, and it never looks to check it. If you move items
by hand, put everything back on the table and type `reset`. `state` prints
the state, `--step` waits for Enter before every action.

### The camera window

Both consoles open a window with the wrist camera live at the top (colour
and depth) and, below, every view the robot takes during an action,
annotated with the SAM 3 masks, each item's estimated model and score, the
basket and the obstacles, and with what the skills chose to show (the
toolkit's `show_*` pictures). `a` and `d` go to earlier and later views,
`l` back to the newest. Without a display (over `ssh` without `-X`) the
console carries on without it.

### When something goes wrong

| Message | What to do |
|---|---|
| `NotImplementedError ...` | the function it names is not written yet |
| `no arm ... Is the robot running?` | the UR driver is not running, or Play was not pressed on the pendant, or the `ROS_DOMAIN_ID` differs |
| `no camera images on /camera/camera/...` | the RealSense node is not running |
| `... is out of the arm's reach` | the pose is too far from the robot's base |
| `MoveIt is not running`, `move_group did not come up` | see `~/.ros/lab_move_group.log` |

The run directory holds `console.log` (every step), the annotated camera
views, `*_survey.yaml` (the estimated placements) and, with `--save-frames`,
the raw frames for `depth_map.py`.

---

## The toolkit

What the exercise may use from the rest of the code is in
[`lab_replica/toolkit.py`](src/lab_replica/lab_replica/toolkit.py), the
robot passed to the skills
([`autonomy/executive.py`](src/lab_replica/lab_replica/autonomy/executive.py))
and MoveIt ([`lab_replica/moveit_client.py`](src/lab_replica/lab_replica/moveit_client.py)).
None of it needs you to know ROS.

### Frames and units

* Metres, radians and newtons throughout.
* **The world frame** has z up and z = 0 on the table top. Its origin is
  on the table below the robot's base axis, x points towards the board and
  y towards the wall ([`config/cell.yaml`](src/lab_replica/config/cell.yaml)).
* **Poses** are 4x4 homogeneous matrices (numpy). `T_a_b` is frame b in
  frame a, so `T_a_c = T_a_b @ T_b_c` ([poses in ROS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)).
* **The gripper's frame** is `rg6_base_link`, the RG6's mounting face. For
  grasps it points straight down, its yaw is the direction of its x axis,
  and the fingers close along its y axis. The fingertips are
  `rg6.depth_from_width(width)` below it.
* **The camera's optical frame** has z along the view, x to the right in the
  image and y down ([REP 103](https://www.ros.org/reps/rep-0103.html#suffix-frames)).
  A pixel (u, v) is column u and row v, so images are indexed `[v, u]`.
* **The arm's joints** are six angles, `ARM_JOINTS` = `shoulder_pan_joint`,
  `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`,
  `wrist_3_joint` ([UR5e kinematics](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)).

### Data types

| Type | Fields | Meaning |
|---|---|---|
| `Frame` | `rgb` (H, W, 3) uint8, `depth` (H, W) float32 metres along the view (NaN where unknown), `K` 3x3 intrinsics, `T_world_cam`, `q` the arm's joints | one RGB-D image with the camera's pose |
| `Placement` | `item`, `rest`, `x`, `y`, `yaw`, `support` (`"table"` or `"basket"`), `z_support` | where and how an item rests |
| `Estimate` | `placement`, `plausibility` (0 to 1), `n_points`, `top_z`, `notes` | an estimated placement |
| `Detection` | `item`, `score` (SAM 3 score x plausibility), `sam_score`, `estimate`, `mask`, `frame`, `geometry_frame` | an item found in a frame |
| `Scene` | `detections` `{item: Detection}`, `basket_xy`, `frames`, `depth`, `obstacles` (each with `polygon`, `z_top`, `support`, `unseen`) | what the robot's looks found (from `robot.survey`) |
| `ImageGrasp` | `u`, `v`, `angle` (radians from +u towards +v), `width_px` | a top-down grasp in an image |
| `Grasp` | `pose` (of `rg6_base_link`), `width`, `tips_z`, `top_z` | a top-down grasp in the world |
| `Held` | `item`, `rest`, `width`, `tip_z` (fingertips above the item's support), `offset_obj` (grasp centre in the item's rested frame), `yaw_rel` (item yaw minus gripper yaw), `top` | how the gripper holds an item |
| `Box`, `Cylinder` | `Box(name, c, R, h)` (centre, rotation, half sizes), `Cylinder(name, c, r, h)` (centre, radius, half height, vertical) | obstacles for MoveIt |
| `ActionError`, `PlanError` | | raised when an action cannot be performed, and when no motion is found |

### Item models

`robot.items[name]` is the model of an item (`lab_replica/objects/`, one
file per kind). Its sizes come from
[`config/items.yaml`](src/lab_replica/config/items.yaml).

| Attribute | Meaning |
|---|---|
| `name`, `kind`, `spec` | its PDDL name, its kind (`block`, `can`, ...), its entry in `items.yaml` (e.g. `spec["size"]`, `spec["stable_poses"]`) |
| `poses[rest]` | a resting pose (`RestPose`) with `rpy` and `z` (how the item frame sits on the support), `top` (its height), `footprint` (its outline in the rested frame), `grasps` (each with `width`, `tip_z`, `yaws`, `continuous`, `center`) |
| `footprint_world(rest, x, y, yaw)` | its outline on the support at a placement |
| `shapes` | its collision shapes (boxes and cylinders) |

The basket (`robot.basket`) has `height`, `floor` (its floor's thickness),
`r_top` (its radius at the rim) and `inner_radius(z)`.

### Toolkit functions

**Geometry and kinematics**

| Function | What it gives |
|---|---|
| `fk(q)` | the world pose of `rg6_base_link` at the joints q |
| `ik_all(T)` | every joint solution that puts `rg6_base_link` at T, each angle in [-pi, pi] ([inverse kinematics](https://moveit.picknik.ai/humble/doc/concepts/kinematics.html)) |
| `joint_limits()` | `{joint name: (lower, upper)}` |
| `gripper_pose(x, y, z, yaw)` | the pose of `rg6_base_link` pointing straight down at height z |
| `look_at(eye, target, up=(0, 0, 1))` | the camera pose at `eye` looking at `target` |
| `down_view(x, y, height, up=(0, -1, 0))` | the camera pose straight above (x, y) looking down |
| `hand_eye()` | the camera's optical frame in the gripper's frame |
| `box_in_world(box, q)` | a box given in the gripper's frame (e.g. `held_box`) in the world, with the arm at q |
| `object_box(model, xyz, R, name)` | the box around an item's collision shapes with its frame at (xyz, R) |
| `placement_pose(model, placement)` | the item frame (xyz, R) of a placement |
| `finger_footprints(x, y, yaw, open_width)` | the two fingers' outlines seen from above |
| `polygons_intersect(a, b, margin=0)` | whether two convex polygons come closer than `margin` |
| `polygon_inside_rect(poly, x0, x1, y0, y1)` | whether a polygon is inside a rectangle |
| `rpy_to_matrix(roll, pitch, yaw)`, `wrap(angle)` | a rotation matrix, an angle wrapped to [-pi, pi) |
| `rg6` | the gripper. `MAX_WIDTH` (its widest opening), `depth_from_width(w)` (fingertips below the mounting face), `free_height(w)` (how high above the fingertips the space between the pads is free of the knuckles) |

**Perception**

| Function | What it gives |
|---|---|
| `rect_fit(xy)` | the minimum-area rectangle around 2D points (centre, (long side, short side), direction of the long side) |
| `robust_top(z)` | the 97th percentile of heights (the top without stray points) |
| `gauss(err, scale)` | exp(-(err / scale)^2), an error turned into a score from 0 to 1 |
| `mask_overlap(a, b)` | the intersection of two masks over the smaller one's area |
| `model_height_map(item, rest, cell)` | the item's top surface seen from above, `.at(u, v)` its heights at points of its rested frame (0 off the item) |
| `estimate_item(...)` | your `estimate_placement`, as the framework calls it |
| `measure_item(pts, table_z, shape=None)` | a new item's (shape, size, notes) from its points |
| `make_item(name, spec)` | an item model from a spec (as in `items.yaml`) |

**Plans.** A plan is a list of steps that `robot.run` performs in order.

| Function | What it gives |
|---|---|
| `trajectory(label, path, slow=False, slow_last=None, slow_first=None)` | a step that moves the arm along a joint path, timed within the robot's limits (the last or first so many metres slowly) |
| `gripper_step(label, width, force, expect_grip=False, drop=None)` | a step that moves the fingers to `width` with at most `force`. With `expect_grip` it fails unless they stop on something |
| `spread(spots, n=12, min_dist=0.03)` | up to n of `robot.drop_spots`' spots, best first, apart from each other |

**Pictures** for the camera window (each also saved in the run directory).
`show_image_grasps(robot, frame, item, mask, grasps)`,
`show_grasp(robot, frame, placement, held, open_width)`,
`show_table_spot(robot, frame, scene, region, target, source)`,
`show_drop(robot, frame, scene, drops, target, z_release)`,
`show_new_item(robot, frame, mask, model, placement, label, title)`,
`show_masks(robot, frame, [(label, mask), ...], title)` and
`show_detections(robot, frame, scene, title)`.

### The robot

The skills get the robot as their first argument. The full list with
details is at the top of
[`exercise/skills.py`](src/lab_replica/lab_replica/exercise/skills.py) and
[`exercise/misc.py`](src/lab_replica/lab_replica/exercise/misc.py).

| Method or attribute | What it does |
|---|---|
| `survey(view, scene=None, held=None, exclude=())` | moves the camera to a view (six joints, a 4x4 camera pose, or a list of them, the first reachable) and recognises what it sees, added to `scene` if given. Returns the `Scene` |
| `holding()`, `width()`, `current()` | whether the gripper senses a grip, its opening, the arm's joints |
| `held`, `now_holding(held, source)`, `now_empty()` | what the gripper holds as far as the skills know |
| `best_frame(scene, item=None)`, `grasp_mask(frame, scene, item)` | the frame to grasp from, the item's pixels there |
| `set_obstacles(scene, exclude=())` | MoveIt's planning scene from your `scene_obstacles` |
| `table_region(basket_xy, depth)`, `drop_spots(scene, held, yaws)` | where items may go on the table, the basket's spots and pile heights |
| `run(steps)`, `placed(item, placement)` | perform a plan's steps, report an item put down |
| `segment(frame, prompt)`, `points(frame, mask)`, `points_around(...)` | SAM 3's masks of a text prompt, the world points of a mask |
| `add_item(name, spec)`, `remove_item(name)` | know a new item, forget it |
| `log(text)`, `phase(text)`, `show(frame, draw, title)` | the log, the status line, a picture |
| `items`, `basket`, `table_z`, `force`, `mv`, `cell`, `cfg` | the item models, the basket, the table's height, the grip force, MoveIt, `cell.yaml`, `perception.yaml` |

### MoveIt

[MoveIt](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) is the
motion planning framework of ROS. It runs as a separate program,
[`move_group`](https://moveit.picknik.ai/humble/doc/concepts/move_group.html),
which the consoles start. `robot.mv` talks to it and takes and returns numpy
values. It plans for the robot model (the arm and the RG6 with its fingers
at the given width) and keeps clear of its
[planning scene](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html),
the table, the wall and the obstacles of `scene_obstacles`. `held` is the
box of the item in the gripper (`held_box`), which moves with it.

| Method | What it gives |
|---|---|
| `plan_to_joints(q_start, q_goal, width, held=None)` | a collision-free joint path, found by [RRT-Connect](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html) (a sampling-based planner, always a way if it finds one, not always a short one) |
| `plan_to_pose(q_start, T_goal, width, held=None)` | the same to any configuration that puts the gripper at `T_goal` |
| `cartesian_path(q_start, [T, ...], width, held=None, min_fraction=1.0)` | the gripper along [straight lines](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html#cartesian-paths) through the poses |
| `is_valid(q, width, held=None)` | whether the robot touches nothing at q |
| `contacts(q, width, held=None)` | what touches what at q |

A joint path is a list of six-angle arrays starting at `q_start`. Every
method raises `PlanError` when it finds nothing. MoveIt's log is
`~/.ros/lab_move_group.log`.

### Configuration

| File | What it holds |
|---|---|
| [`config/cell.yaml`](src/lab_replica/config/cell.yaml) | the table, the board, the wall, the robot's home joints, the camera's pose on the gripper, the grip force, where items may be placed |
| [`config/items.yaml`](src/lab_replica/config/items.yaml) | the twelve items' kinds and sizes |
| [`config/perception.yaml`](src/lab_replica/config/perception.yaml) | the camera topics, SAM 3's text prompts for each item and the basket, score thresholds |

### Further reading

* ROS 2 Humble, [the basics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html) and [frames and transforms](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)
* MoveIt 2, [concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) and [motion planning](https://moveit.picknik.ai/humble/doc/concepts/motion_planning.html)
* PDDL, [domains](https://planning.wiki/ref/pddl/domain) and [problems](https://planning.wiki/ref/pddl/problem)
* [The Unified Planning Framework](https://unified-planning.readthedocs.io/en/latest/) and [Fast Downward](https://www.fast-downward.org/)
* [SAM 3](https://huggingface.co/facebook/sam3), segmentation from a text prompt
* [The pinhole camera model](https://en.wikipedia.org/wiki/Pinhole_camera_model) and [the RealSense's projection](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0)
* [OpenCV's contour features](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html) (`minAreaRect`, `convexHull`)
* [The UR5e](https://www.universal-robots.com/products/ur5e/) and [the RG6](https://onrobot.com/en/products/rg6-gripper)
