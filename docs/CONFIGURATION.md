# Shared configuration and desktop controls

Edit `config.yaml` in the project root, or use **Edit config.yaml** in either
desktop panel. All distances are metres and angles are radians. Settings apply
to the existing `ur5` + right `hand_g` / `G1M5` assembly; changing robot type or
hand hardware still requires matching vendor launch and controller files.

## Object and environment

The default target is `tofuprintfinal`, imported from GraspIt. Its original
WRL/XML and converted STL are in `assets/models/tofuprintfinal/`. The config
uses the STL for both appearance and collision with `mesh_scale: [0.001,
0.001, 0.001]` (source millimetres). `object.pose` rotates GraspIt +Y up to
ROS +Z up and places the mesh on the floor. The runtime name stays `rect_object`
for planner/replay compatibility. Mass is 0.5 kg and the object remains static.

Set `collision_mesh: true` to use the visual mesh for collisions in Gazebo and
optional real MoveIt. Set it to `false` for the `collision_size` box. `mesh: null`
uses the `visual_size` box and requires `collision_mesh: false`. All model paths
must be inside the project. `model` is the SDF template; shared config overrides
its geometry, mass, material and static setting.

`object.grasp_z_offset` is an extra world-Z translation in metres, currently
`0.02` (2 cm). It shifts only the object and the resolved hand grasp target,
in both manual and GraspIt capture modes. It leaves the robot base/home joints,
floor, walls, hand angles and lift distance unchanged. The YAML base poses and
source TXT stay unchanged; each new recording stores the shifted grasp and
effective object pose. Old trajectories retain their original positions.
Restart simulation or use Reset simulation to place the object at the new
height before planning a new run.

Set **`object.hand_collision: false`** to let the hand pass through the object in
Gazebo and to allow that same pair in MoveIt during stages 2–5.
**Planning Stage 1 always checks hand-object collisions in MoveIt**, including the IK
search, hand preparation and arm approach. The configured rule is restored
after Stage 1, including on failure. Lift preflight uses the stages 2–5 rule.
Simulation replay disables MoveIt collision checking for all stages.
Real replay always checks robot-table collisions, including the arm, hand,
attached objects, planned moves to trajectory starts and replanned fingers.
Saved arm paths are checked at their waypoints and between points (maximum
joint increment 0.01 rad) before execution. Real replay refuses to start if
`ground_plane_user` is absent from the planning scene. Other collision pairs
remain disabled during replay. On exit, previous rules are restored with
robot-table checking retained in real control. Replay still checks joint
limits, trajectory files and starting positions. Gazebo physical contacts and
hardware controller protections are unaffected.
Set the option to `true` to check hand-object collisions in all planning stages.
This stage-specific override changes planning checks; Gazebo physical contact
filtering still follows the option for the entire session.
Restart simulation after changing this setting: Gazebo loads
the physical filter with the world. The desktop Start button applies the
MoveIt rule, and each new plan reads it again. Check-only replay remains
read-only. The switch targets the `rh_` hand links and the object; it keeps the
object's collision geometry and its Gazebo contacts with the arm and floor.
Other MoveIt collision pairs remain checked throughout planning.

On real hardware this controls MoveIt checking when the object is present in
the planning scene (`add_to_real_scene: true`, or added externally). It cannot
disable physical contact forces on the real hand. Gazebo's filter is compiled
automatically inside the existing Shadow image; the host needs no compiler.
The filter uses Gazebo Classic's [collision bitmasks](https://get.gazebosim.org/tutorials?cat=physics&tut=collide_bitmask).

`simulation.physics_step` controls the Gazebo ODE step in seconds. The default
is `0.00025` (0.25 ms), tested to reduce closed-finger shaking during arm motion.
The generated world sets `real_time_update_rate` to `1 / physics_step` (4000 Hz
at this setting). Reset/restart simulation to apply changes. Smaller steps use
more CPU; the actual real-time factor depends on available compute. `0.001` is
the previous 1 ms setting. This does not change the real robot's controller loop.
See [Gazebo physics parameters](https://classic.gazebosim.org/tutorials?tut=physics_params).

`simulation.world` selects `assets/worlds/open.world` or
`assets/worlds/cell.world`. The `environment` section sets obstacle sizes and
poses. Simulation updates obstacles included in the selected world; the open
world keeps only the ground, and the cell world includes the surrounding walls.
Real startup uses the same `simulation.world` selection and configured sizes/poses.
It removes configured obstacles absent from that world, including stale cell walls
when switching to the open world. Restart real control after changing the selection.
The real-control panel loads these objects as soon as MoveIt's scene services
are available, before waiting for the pendant's External Control program.
The object's
collision geometry is added to real MoveIt only when `add_to_real_scene` is true.

Simulation startup writes a world and object SDF under `.runtime/`; the original
assets remain unchanged. Startup also synchronizes generated models into the
container’s gazebo2rviz model lookup, so MoveIt reads the configured mesh rather
than the legacy box. The planner keeps the object in the scene throughout
the task and applies the configured hand collision rule. Restart after scene or robot placement
changes. `open_no_collision.world` is also accepted as a base world, but object
collision geometry follows `collision_mesh` and `collision_size`.

## Five-stage grasp task

`robot.initial_z` is the single UR5 mounting height in metres above world Z=0
(the table top). It is currently `0.012`; set `0.032` to raise the UR5 base by
another 2 cm. Only the robot placement changes: the object, world grasp target, floor,
walls and home joint angles keep their coordinates. The mounted hand follows
its arm. This is independent of `object.grasp_z_offset`. Restart the session
after changing the base height so Gazebo and the combined planning model agree.

The arm home angles remain in `robot.home_joints` (radians). These are used both
for simulation startup and for the final return home.

`grasp.mode` selects the source. The current config enables capture loading:

```yaml
grasp:
  mode: graspit                       # manual uses the YAML pose/joint maps
  capture_file: grasps.json             # JSON or legacy TXT, relative to project root
  capture_index: 0                     # zero-based JSON record index
  capture_tighten_degrees: 7.0          # stage 3 J2 increment, degrees
  frame: world
  hand_link: rh_palm
```

JSON capture input uses schema version 1 from the modified GraspIt Capture
window. `capture_index` selects a record from `grasps`. The importer reads
`hand_pose_world`, `object_pose_world`, `gravity_direction_world`, and actual
`joints[].joint_value_rad` when provided. Fixed DH `angle_rad` offsets are
already represented by the ROS model and are not added to joint commands.
Small export-rounding differences at joint limits are clamped (up to 1e-5 rad
for JSON). Larger limit violations stop loading.

The captured world-frame hand pose is first transformed into the captured
object frame, then placed using the configured `object.pose`. Millimetres
convert to metres and the hand-base transform converts to `rh_palm`. The saved
gravity direction is transformed into the palm frame, so moving/rotating the
object or changing world coordinates preserves its relationship to the grasp.

Legacy TXT remains supported (first `5` + 13-DOF line, then `0` + XYZ + WXYZ
pose). It assumes the original object-relative pose format. It has no saved
gravity vector; disable lift alignment or provide an explicit
`grasp.gravity_direction_palm` vector to use a legacy/manual pose with alignment.
All captures must be inside the project so the containers can read them.

Capture mode overrides the YAML `translation`, `orientation`, `joint_angles`
and `tightened_joint_angles` in memory. Stage 3 adds `capture_tighten_degrees`
to FF/MF/RF J2 and THJ2, the joints immediately before the fingertip joints.
J1 targets retain their captured angles. The importer supports the exported `shadowhand_motor`
model and rejects missing, malformed or out-of-range data without manual fallback.
Only the added tightening is capped to the available joint range. For example,
THJ2 already at +40° stays there when +7° is requested; other joints receive the
full increment where possible. Planning reports reduced increments and stores
them in `task.json` as `tightening_limited_joints`. Captured angles and explicit
manual targets still undergo their usual joint-limit validation.

Switch to `mode: manual` to use the explicit YAML pose and both joint maps.
Omitting `mode` also preserves the original manual behavior. Generate a new
plan after changing the capture: each run stores the resolved pose, joint
targets, source filename and SHA-256 hash in `task.json`; replay uses its saved
targets rather than reinterpreting the source capture.

The manual pose and hand configurations are kept below as an editable fallback:

```yaml
grasp:
  frame: world
  hand_link: rh_palm
  translation: [0.34100592749911035, -0.010426620623213072, 0.09594225828885045]
  orientation: [0.4026467431013437, 0.49784906485097036, 0.5312893123372578, 0.554755419525667]
  joint_angles:                 # Stage 2, radians
    # ... full joint map from config.yaml ...
  tightened_joint_angles:       # Stage 3, absolute radians
    # ... another complete map ...
```

The translation is the **palm** position in world coordinates, in metres.
Orientation is a quaternion `[x, y, z, w]`. The planner reads the fixed
flange-to-palm transform from the robot model and converts this hand pose to the
UR5 flange target. The values reproduce captured grasp `261110.txt`. Frame and joint mapping details
are recorded in [the import notes](../assets/grasps/261110/README.md).

The stage-3 map starts from the stage-2 map, with **0.1745329252 rad (10°)** added
to `rh_FFJ1`, `rh_MFJ1`, `rh_RFJ1` and `rh_THJ1`. Other joints are unchanged.
The maps are independent absolute targets: edit `tightened_joint_angles` to
change stage 3. Both maps must name the same joints. Omitted active hand joints
are set to zero. Unknown joints, nonfinite values and joint-limit violations
fail before stage 1 rather than silently clamping a target.

The five stages are thumb-base preparation plus arm approach, close, tighten,
lift, then arm home and hand zero. At the start of stage 1, `rh_THJ5`, `rh_THJ4`,
`rh_FFJ4`, `rh_MFJ4`, and `rh_RFJ4` move to their full grasp angles while the
other joints move 60% from current angles toward their grasp targets
(`planning.approach_hand_fraction: 0.6`, range 0–1). The arm
then approaches; stage 2 applies the remaining captured joint angles. Replay
preserves this order. New recordings save the complete preparation target and
replay it exactly, marked by `hand_preparation_mode: saved_target` in `task.json`.
Earlier recordings without that field retain live-angle thumb-only preparation.
Older recordings without `thumb_preparation_joints` retain their original `rh_THJ5`-only preparation.
Before motion, the approach search holds the prepared hand posture and tries
`planning.approach_ik_attempts` starting guesses (default 64), with
`planning.approach_ik_timeout` seconds per guess (default 0.1). Every distinct
IK configuration is checked for whole-robot collisions under the configured
collision rules. Before any task motion, the closed and tightened hand states
and the Cartesian lift (including gravity alignment) are checked for each
candidate. Configurations without a valid lift are discarded. Remaining goals
are ranked by joint distance from the current arm pose.
Stage 1 plans to these joint goals in order, trying the next if no path
is found. Only a successfully planned path is executed; an execution failure
stops the task. No IK result bypasses path collision checking. Search failure
reports observed contacts, rather than claiming that every configuration is
impossible. Stage 4 plans the lift again from the measured state to account for
tracking differences. Saved trajectories and replay retain the selected arm path.
Stage 5 opens the hand only
after the arm has successfully returned home.

`planning.lift_distance` is currently **0.20 m** along world +Z. With
`planning.lift_align_gravity: true`, stage 4 simultaneously rotates the palm
so the saved gravity direction points toward world `[0, 0, -1]`. It uses the
shortest alignment rotation and smooth quaternion interpolation throughout
the lift. Palm X/Y stay fixed; the planner compensates for the fixed palm-to-
flange offset at every waypoint. Finger angles hold the stage-3 target.

`lift_step` limits palm translation sampling; `lift_rotation_step_degrees`
(default 2°) limits angular sampling. Alignment requires a complete 100% path,
including collision checks, even if `lift_min_fraction` is set lower. Set
`lift_align_gravity: false` for the previous fixed-orientation lift. Endpoint
IK is checked before any task motion; Cartesian collision/IK failures stop the
lift rather than executing a partial rotation.

The path is retimed under `lift_velocity`/`lift_acceleration` (each defaults to
the corresponding arm scaling when omitted) and stretched to at least
`lift_min_duration`. Current lift defaults are 0.10 for both scalings and a
16-second minimum; other arm stages keep 0.08 scaling. A short minimum duration
cannot speed it beyond those limits. `task.json` saves the resolved gravity direction, source hash,
alignment target and lift settings. Replay executes the saved arm trajectory,
including rotation, and keeps the existing Enter-per-stage prompts. Older
recordings retain their original lift behavior. Generate a new plan to record
the gravity-aligning lift.

Other planning settings define arm/hand speed scaling, planning time/attempts
and joint tolerance. `network` defines workstation/NUC/UR5 addresses, SSH key
path and ROS port.

`trajectories.replay_run` now defaults to `latest`, selecting the latest
completed **five-stage** recording by creation time. Set a saved folder name to select a
specific compatible run. Old recordings such as `2026-11-10_13-35-00` remain
unchanged and cannot be replayed as the new task. Use **Plan, execute & save**
to create a new recording first. Config edits do not retarget saved runs; their
hand targets, home and lift settings are captured in `task.json` and saved data.

`.env` holds Docker images, host UID/GID, display and graphics options. Network
definitions in `config.yaml` supersede network values in a legacy `.env`.
Explicitly exported environment variables such as `PC_IP` still override YAML.
Each session captures its Docker/network settings in `.runtime/` so shutdown
uses the NUC address and SSH identity with which that session started, even if
configuration is edited while it runs. Do not delete `.runtime/` during a session.

## Shortcuts

In Ubuntu's **Files** app, open **Home → shlite_complete**, right-click
**`Simulation.sh`** and choose **Run as a Program**. Use **`Real-Control.sh`** for
the real-hardware panel. Both executable scripts are included in the project root.
They locate the adjacent `robotctl`
from their own location, including paths with spaces, and continue to work when
the project is moved or copied. There is no installation or regeneration step
after moving.
The optional `./robotctl shortcuts` command regenerates
only these project files and never writes to the Desktop or application menu.
The launchers only open a panel; hardware starts when **Start real hardware**
is clicked. Simulation and real sessions cannot be run together.

Startup logs appear inside the panel. Replay opens a terminal so each stage can
be started with Enter, or the task quit with q between stages. **Shut down
simulation** stays enabled even during startup or while a task terminal is
open. It cancels the terminal wait and stops the container without requiring
Enter or closing that terminal. Repeated clicks are disabled while shutdown
is already running. Real hardware still requires finishing/quitting the task
and closing its terminal before its calibration shutdown. **Plan, execute & save** creates a
new folder with `arm/` and `hand/` subfolders. GraspIt mode uses the input filename
stem, with `_001`, `_002`, etc. for repeated saves; manual mode uses Tokyo timestamps.
Failed or interrupted planning runs remain archived; default replay skips them.
**Reset simulation** shuts down and recreates the simulation session at its
configured starting state: arm home, initial hand posture and object/world
poses (including the offsets). It reloads config and starts fresh controllers.
Recordings are kept. The normal readiness and collision checks run again.
Reset stays available during startup, planning, replay, failed tasks and terminal
prompts. It cancels the current task, stops the simulation and restores the
starting session. Shutdown remains available during reset and cancels its restart.
Only a reset/shutdown already in progress disables another reset click.
A failed startup leaves the partial session
available for shutdown; a failed hand shutdown calibration leaves the session
running for inspection and retry.

Host dependencies on Ubuntu: `python3-yaml`, `python3-gi`, `gir1.2-gtk-3.0`,
`x11-xserver-utils`, `xdg-utils`, and `gnome-terminal`, in addition to Docker
Engine and Compose. Host-side checks, without Docker or robot connections:

```bash
./robotctl validate-config
python3 -m unittest discover -s tests -p test_desktop_workflow.py
```

The **Saved trajectories** dropdown selects the run for both **Check saved task**
and **Replay five-stage task** in simulation and real control. It starts from
`trajectories.replay_run`, then keeps your panel selection without rewriting
config. The selected timestamp is passed explicitly to replay. **Latest
completed run** resolves to a specific folder when the task starts. The list
refreshes automatically while idle and with **Refresh list**. Failed, incomplete,
missing-file and older-sequence recordings are labeled and cannot execute;
only complete compatible runs enable Check/Replay. No trajectory pickle is
loaded just to populate the dropdown.
