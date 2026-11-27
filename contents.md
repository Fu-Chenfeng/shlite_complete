# Project details and usage

For installation and a quick start, see [README.md](README.md).

- [Configuration and launchers](#configuration-and-launchers)
- [Simulation](#simulation)
- [Custom trajectories](#custom-trajectories)
- [Five-view camera tour](#five-view-camera-tour)
- [Real arm and hand](#real-arm--hand)
- [Shared trajectories and sequence](#shared-trajectories-and-sequence)
- [Project contents](#project-contents)


UR5 + four-finger Shadow Hand Lite (hand_g G1M5): Gazebo simulation, simulation
planning/saving, real control through the existing hand NUC, and one shared replay
workflow. Linux with Docker Engine, Docker Compose v2 and an X11/XWayland desktop.

## Configuration and launchers

The shared settings are in **[`config.yaml`](config.yaml)**: object model/mesh,
pose, dimensions, color, environment obstacles, robot startup/home pose, network
addresses and planning targets. `.env` contains Docker images, host UID/GID and
display settings. See [CONFIGURATION.md](docs/CONFIGURATION.md) for examples.

Grasp source: `grasp.mode: graspit` reads `grasp.capture_file` (a project-relative TXT or JSON file). Set `mode: manual` to use the YAML pose and
joint maps. JSON capture mode also reads the saved optimum gravity direction.
Capture mode maps the hand relative to `object.pose` and uses
`capture_tighten_degrees` for the stage-3 J2 bend (the joint before each tip). Generate a new plan after
changing the capture. See [configuration](docs/CONFIGURATION.md).

Set `robot.initial_z` to the robot mounting height above the table (metres;
currently `0.012`, restart to apply), independently of the object/grasp.

Set `object.grasp_z_offset: 0.02` to raise only the object and grasp target by
2 cm. Reset simulation recreates the session at arm home and its initial hand and
world/object positions, including this offset.

`object.hand_collision` controls Gazebo physical contacts and planning stages
2–5. Planning Stage 1 always checks hand-object collisions. Simulation replay
disables MoveIt collision checks; real replay retains robot-table checks and
requires the table model in its scene. See [collision settings](docs/CONFIGURATION.md).
Restart simulation after changing physical contact settings.

```bash
cd ~/shlite_complete
./robotctl init
# Review config.yaml (especially network) and the generated .env.
./robotctl gui
./robotctl verify-images
```

In Ubuntu's **Files** app, open **Home → shlite_complete**, right-click
**`Simulation.sh`** and choose **Run as a Program**. For real control, do the same
with **`Real-Control.sh`**. These executable launchers ship inside the project and find `robotctl` beside them,
so they work after copying or moving the folder without installation or path
updates. Each opens a native control panel with
Start, task check, step-by-step Replay, Status, Edit config.yaml and Shut down
buttons and a Saved trajectories dropdown. Simulation also offers Pick up
trajectory and Reset simulation. No commands need to be
typed for everyday sessions. The optional `./robotctl shortcuts` command only
regenerates the launchers inside the project; it writes no external shortcuts.

Both panels have a **World** dropdown with **tofu**, **tofu no collision**,
**tofu cell**, and **no object**. The **no object** world uses the tofu scene
without its tofu object, in both Gazebo and the real MoveIt scene.
Choose before starting: simulation loads that Gazebo
world, and real control uses its environment in the MoveIt scene. The choice
is kept for that panel's session, including simulation resets; it is not stored
in `config.yaml`. Shut down the session to choose another world.

Start real hardware launches ROS core, the NUC hand loop, UR5 driver, adapter and
MoveIt in order, waiting for readiness. Keep the hand clear during calibration;
start the External Control program on the pendant when prompted. Startup never
replays a task. Replay opens a terminal for Enter/q stage prompts. Simulation
shutdown and Reset stay available during startup, planning, replay and terminal prompts;
it cancels the task wait and stops the simulation container. For real hardware,
finish or quit the task and close its terminal before shutdown. Keep the panel
open until shutdown completes; real shutdown retains the original hand calibration.
SSH access must already work (unlock an encrypted key in your SSH agent first).

Host desktop controls require `python3-yaml`, `python3-gi`, `gir1.2-gtk-3.0`,
`xdg-utils` and a terminal such as `gnome-terminal`. The command-line workflow
below is also available; it requires `python3-yaml` but does not require GTK.

For image transfer and new-PC hardware setup, see
[TRANSFER.md](docs/TRANSFER.md) and [NEW_MACHINE.md](docs/NEW_MACHINE.md).
`up` starts idle workstation containers; hardware starts only with the explicit
`real nuc` and `real ur5` commands. Stop the old project's sessions before use.
The saved runs in `trajectories/` are included in the repository. Validation scope is recorded in [VALIDATION.md](docs/VALIDATION.md).

## Simulation

**Terminal 1:**

```bash
cd ~/shlite_complete
./robotctl sim up
./robotctl sim start
```

Wait for Gazebo, RViz and MoveIt. **Terminal 2:**

```bash
cd ~/shlite_complete
./robotctl sim plan       # plan, save and execute the five-stage task in Gazebo
```

Planning inputs are in `config.yaml`: `robot.home_joints` for the arm home,
`grasp` for the world-frame palm pose and both hand joint targets, and `planning`
for motion settings including the currently configured 0.20 m lift.
GraspIt plans save to a folder named after the input file, e.g. `trajectories/grasps/`.
Repeated saves add `_001`, `_002`, etc. Manual plans use `YYYY-MM-DD_HH-MM-SS`
(Tokyo time). Each folder has `arm/` and `hand/` inside; earlier runs are preserved.
Replay uses the latest completed five-stage run by default; set `trajectories.replay_run`
in `config.yaml` to a saved folder name to select another five-stage run.
To replay the currently saved task instead of generating another:

```bash
./robotctl sim check
./robotctl sim replay     # Enter before each stage; q quits between stages
# Or: ./robotctl sim run  # all stages, sequentially, without prompts
```

Shutdown, from another terminal:

```bash
./robotctl sim shutdown
```

The default **tofu** world is `assets/worlds/open.world`, copied from your usual
`my_world_no_ob.world`. To use the cell-obstacle variant:
`./robotctl sim start world:=/project/assets/worlds/cell.world`.

## Custom trajectories

Start simulation, wait for readiness, then click **Custom trajectory**.

1. On the left, **+** appends a task and **−** deletes the selected task. A new
   task copies the previous task's joint angles. The first task uses the live
   robot state when the editor opens.
2. On the right, **Capture RViz state** records the editable goal pose shown in
   RViz's MotionPlanning display, including both arm and hand joints. Set the
   arm/hand goals using RViz's interactive markers or Joints tab before capture.
   **Capture live state** records the simulated robot's actual joint feedback.
   Select a recording to inspect its angles.
3. Double-click a task to open its joint editor. All angles are in **degrees**,
   with joint limits shown in tooltips. Select a recorded state, then click
   **Read hand goal** or **Read arm goal** to copy only that group's angles.
   Edit any angle manually and click **Apply**. Choose whether the task moves
   the arm before the hand or the hand before the arm.
4. Set the trajectory name, then click **Plan, execute & save**. The entire
   sequence is planned from consecutive endpoints before execution starts.
   **Plan & save** saves the planned sequence without moving the robot.

The default name is `custom_YYYY-MM-DD_HH-MM-SS` in Tokyo time. Runs are saved
under `trajectories/<name>/`, with suffixes to preserve existing runs. Each run
contains arm/hand trajectories, saved hand targets and a `task.json` with the
task sequence, captured states and planning settings. Custom runs appear in the
main **Saved trajectories** dropdown and use the same **Check saved task** and
**Replay saved task…** controls; replay prompts once per custom task.

Restart an existing simulation after installing this feature to load the RViz
goal bridge. The bridge builds inside the simulation container at startup.

Command-line equivalents:

```bash
./robotctl sim custom-capture --source rviz
./robotctl sim custom-capture --source current
./robotctl sim custom-plan --sequence /project/path/to/sequence.json --execute
```

Sequence JSON uses `version: "custom_v1"`, `name`, and a nonempty `tasks` list.
Each task has `name`, `order` (`arm_hand` or `hand_arm`), and complete `arm` and
`hand` joint dictionaries in **radians**. Saved `task.json` files can also serve
as sequence inputs. Collision checking uses the current MoveIt scene.

## Five-view camera tour

In the simulation panel, click **5 views + home**. It plans the entire sequence
before moving, then executes and saves it:

1. Close the hand to the camera fist posture.
2. Move the arm home, then visit camera azimuths **0°, 72°, 144°, 216°, 288°** in order.
3. Return the arm home, keeping the fist closed throughout all arm movements.

The fist is defined by `camera_scan.fist_joint_angles`, independently of the
object's grasp settings. The planner checks the closing motion first, then
uses its end state for every arm IK and path check. The complete sequence is
planned before execution begins.

All segments use collision checking, including hand-object checking even when
`object.hand_collision` is false. Several IK configurations are tried at each
view; if a later segment or return home fails, the planner tries another branch.
If no complete route is found within the search budget, it stops before motion.
Reset and shutdown remain available during the task.

`camera_scan` in `config.yaml` controls the orbit distance (0.4 m), elevation
(30°), yaw, image roll, pause at each view, IK attempts and planning budget.
`roll_degrees: 180` turns each view upside down around its optical axis while
preserving the optical center and viewing direction. Set it to `0` for the
original DefMapSim image orientation. Camera calibration
is relative to **ra_flange**: `mount_translation` is the depth optical center;
`mount_forward` and `mount_up` define its viewing direction and roll. The default
assumes the optical center at `[-0.02, -0.08, 0]` m in flange coordinates,
with the camera facing along the hand. This moves the previous mount 2 cm
toward the wrist and 2 cm farther to the side, preserving the exact 0.4 m
camera targets and 180° image roll. The
`object_center_local` default is the tofu model's convex-hull center in metres;
change it when changing the model. The world-space target includes the object's
pose and `grasp_z_offset`. `reference_offset_right` reproduces DefMapSim's calibrated
3.5 cm offset from its camera reference to the depth optical center.

The complete fist-held tour and return home have been checked at these exact
targets. See [mount validation](docs/diagnostics/WRIST_CAMERA_EXACT_MOUNT.md)
for the planned endpoint errors and center-sightline checks.

After pressing **5 views + home**, five transparent yellow cones mark the camera views in Gazebo and RViz's
**Camera views** display. Each pointed tip is at the depth optical center;
its circular base extends 12 cm along the camera viewing direction, with a
4 cm radius. These are direction indicators, not calibrated camera fields of
view, and have no collision geometry. Planning updates them to match the
current targets. The cones stay hidden at simulation startup and are cleared
on reset.

Runs are stored as `trajectories/camera_views`, `camera_views_001`, and so on,
with arm/hand trajectory files and `camera_scan.json` containing the ordered
motions, camera poses and settings. They are marked separately in the saved-run
list and are not passed to the five-stage grasp replay. This feature positions
the assumed camera; **it does not yet render or record depth images**, and no
camera/bracket collision geometry has been added.

Optional terminal equivalents:

```bash
./robotctl sim camera-plan --execute  # Same as the button.
./robotctl sim camera-plan            # Plan and save without moving.
```

## Real arm + hand

Connect the laptop to the NUC network and the UR5 network. Check `config.yaml` and NUC
clock synchronization as described in [NEW_MACHINE.md](docs/NEW_MACHINE.md).
Finish and stop the simulation first. Keep the hand clear during calibration.

**Terminal 1:**

```bash
cd ~/shlite_complete
./robotctl real up
./robotctl real core
```

Wait for `started core service [/rosout]`. In **separate terminals**, each from
`~/shlite_complete`, run in order:

```bash
# Terminal 2: real hand hardware; wait for initialization
./robotctl real nuc

# Terminal 3: real UR5 driver; start the usual External Control program on the pendant
./robotctl real ur5

# Terminal 4: arm joint/action adapter
./robotctl real adapter

# Terminal 5: combined arm-and-hand MoveIt + RViz
./robotctl real moveit
```

Leave these terminals running. In **terminal 6**:

```bash
cd ~/shlite_complete
./robotctl real obstacles
./robotctl real check
./robotctl real status
```

Confirm RViz matches the physical assembly and the UR5's scaled trajectory
controller is `running`. Then replay:

```bash
./robotctl real replay    # Enter executes the next stage; q quits between stages
# Or: ./robotctl real run
```

Finish motion and quit replay. Keep the other terminals open, then run:

```bash
./robotctl real shutdown
```

This performs the original hand shutdown calibration, stops all ROS nodes on
this session's master, and stops the NUC and workstation containers. The laptop,
NUC and UR control box stay powered on. If calibration or SSH fails, the command
stops at that failure so you can inspect it. Do not close ROS core first.
If the real core has already exited, shutdown finishes container cleanup without
attempting calibration. Repeating shutdown is supported and does not contact a
running simulation master. After containers stop, the panel closes lingering
local launchers and SSH clients.

## Shared trajectories and sequence

Both modes use **`app/replay.py`** and **`trajectories/`**. Grasp recordings use
the following sequence; custom recordings use their saved task order:

1. Move `rh_THJ5`, `rh_THJ4`, and each finger's J4 to their full grasp targets;
   move other hand joints toward their targets by `planning.approach_hand_fraction`
   (for example, `0.6` means 60%), then move the arm so
   `rh_palm` reaches `grasp.translation` and `grasp.orientation`.
2. Close the fingers to `grasp.joint_angles`.
3. Apply `grasp.tightened_joint_angles` (capture mode: finger/thumb J2 joints +`capture_tighten_degrees`).
4. Lift the palm by `planning.lift_distance` (currently **0.20 m**) along world Z,
   rotating during the lift to align the captured gravity direction with world-down
   when `planning.lift_align_gravity` is enabled.
5. Return the arm to `robot.home_joints`, then set every hand joint to zero.

Replay prompts once before each stage; stages 1 and 5 each complete both actions
in order. New recordings replay the exact saved hand preparation posture.
A failed action stops the sequence. Real arm replay retains the saved timing;
simulation replay scales arm and hand motion with `simulation.replay_speed`
(default 3×). Hand motions use the absolute targets saved with the recording. Later config edits
apply to newly planned runs. Each run contains `task.json` with its five-stage
format and config snapshot; legacy recordings are rejected by this workflow.
See [TRAJECTORIES.md](docs/TRAJECTORIES.md) for files and settings.

## Project contents

| Path | Purpose |
|---|---|
| `config.yaml` | Object, scene, robot, network and planner definitions |
| `Simulation.sh`, `Real-Control.sh`, `app/control_panel.py` | Project launchers and button controls |
| `compose.yaml`, `.env.example`, `robotctl` | Compose services and host commands |
| `app/` | Planner, shared replay, controller adapter and diagnostics |
| `launch/` | Simulation, combined MoveIt and original UR driver launch |
| `assets/`, `config/` | Worlds, models and robot/controller configuration |
| `trajectories/<run>/arm`, `trajectories/<run>/hand` | Named or timestamped saved runs, included in the repository |
| `docker/` | Shadow startup and self-contained UR5 Dockerfile |
| `docs/` | New-machine registration, transfer and trajectory reference |
| `tests/`, `images.lock.json` | Offline validation and tested image identities |

No source directory outside this project is required. The existing NUC retains
its own `dexterous_hand` container, hand calibration and hardware configuration.
This project is a workstation transfer, not a NUC backup.

## License

Original project code and documentation are available under the [MIT license](LICENSE).
Upstream robot configuration and model files retain their original licenses;
see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
