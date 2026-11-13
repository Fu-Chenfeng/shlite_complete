# Migration validation — 2026-11-09

Validated using the image IDs recorded in `images.lock.json`:

- Compose profiles resolve with all project bind mounts inside this folder.
- A temporary Compose simulation service starts, uses the host UID, and can
  write to the shared trajectory directory. Test containers/volumes were removed.
- The combined robot model, `right_arm`/`right_hand` groups and connected
  world-to-palm transforms load in an isolated ROS session.
- Headless Gazebo starts both controllers and MoveIt with the bundled models.
- The complete plan/execute/save workflow passes in `open.world`, the default
  `my_world_no_ob.world`, writing to a temporary directory.
- Planner save-format roundtrips and the original stage order were checked.
- The UR5 launch retains the original driver defaults and scaled controller
  under an isolated model namespace, without starting a driver connection.
- The copied baseline consists of 13 unchanged pickle files.

The included legacy task was not validated through complete replay: an initial
attempt encountered a planning failure at the hand adjustment stage. Further testing of this
legacy dataset is deferred. Replace or regenerate it before treating a replay as validated.

The clean project's real-hardware workflow was not run because the arm and hand
were disconnected. Its driver/adapter architecture comes from the earlier
working setup. No hardware controller was started during this migration.

`tests/` contains offline checks. Run them in a network-isolated container; the
simulation integration check starts Gazebo and its simulated controllers only.
Optional `CHECK_PLANNER=1` and `CHECK_REPLAY=1` enable full simulated execution.

## Configuration and desktop controls — 2026-11-10

- All 34 offline unit checks passed in the existing Shadow image with networking
  disabled, including configuration validation and startup/shutdown ordering.
- All 17 original planner definitions were preserved in `config.yaml`.
- Both GTK control panels rendered and their initial button states were checked.
  The launchers reside in the project root and were checked after relocating to
  a path with spaces. Only executable shell launchers are retained.
- The generated world started in headless Gazebo with simulated controllers,
  MoveIt, the new readiness probe and saved-task preflight passing.
- The configured obstacle sizes and poses were checked in combined MoveIt using
  synthetic joint states and action servers that reject every motion goal.
- No real hardware session or complete task replay was run for these changes.

## Timestamped trajectory archives — 2026-11-10

- Moved the existing 13 pickle files into `trajectories/2026-11-10_13-35-00/`;
  every file's SHA-256 hash was unchanged after the move.
- All 43 offline checks passed, including separate folders per planning run,
  Tokyo timestamps, same-second collisions and failed-run replay selection.
- Combined MoveIt's read-only replay preflight selected and checked the migrated
  run successfully in a network-isolated container with synthetic joint states.
- Removed desktop-entry files and their generator; the two executable shell
  launchers remain in the project root.

## Hand-object collision switch — 2026-11-10

- All 50 offline unit checks passed, including hand-only MoveIt rules, existing
  default-rule preservation and Gazebo's flattened `rect_object__link` name.
- Isolated Gazebo drop tests confirmed that disabling hand-object contacts keeps
  arm-object, other-object, hand-floor and enabled-reference-object contacts.
- Combined MoveIt checked the collision switch in both directions without motion.
- With `object.hand_collision: false`, the full simulated plan executed and saved
  every stage to a temporary timestamped run; its replay preflight passed.
- The test used a separate project copy. Existing recordings, the configured
  replay selection and real hardware were unchanged.

## Five-stage grasp workflow — 2026-11-10

- All 65 offline tests passed for the new sequence, palm/flange conversion,
  absolute grasp targets, +10 degree fingertip defaults, partial-lift rejection,
  home-angle wrapping, hand feedback and versioned recording selection.
- The isolated combined real-control model accepted new-format recording
  preflight and reported the retained grasp's unreachable 0.40 m lift endpoint
  before motion. No hardware goals were sent.
- That earlier pose/lift combination was later replaced by the GraspIt import
  below, preserving the user’s updated 0.20 m lift setting.
- In a temporary simulation with a closer grasp, the complete 0.40 m Cartesian
  path and arm return were exercised. Final hand-zero execution showed Gazebo
  feedback/tracking instability at rh_FFJ4; the full end-state verification did
  not pass. Do not treat complete simulated replay or real hardware as validated.
- Legacy recordings are unchanged. The new workflow requires version-2 runs
  with task.json; it does not reinterpret the old sequence or shorter lift.

## GraspIt 261110 import — 2026-11-10

- Imported authoritative tofuprintfinal WRL/XML from `grasp_planner` and converted
  all 3,134 triangles to STL without changing coordinates or units.
- Registered fixed chain roots against the ROS hand; independently checked all
  16 link origins at the configured posture (maximum error 0.00024 mm).
- Rendered the project hand/object meshes for comparison with the supplied
  screenshot. Preview and calibration are in `assets/grasps/261110/`.
- The isolated combined MoveIt model accepted both hand targets, five-stage replay
  preflight and the current 0.20 m lift endpoint IK. No hardware goals were sent.
- Full motion execution and grasp stability have not been validated for this import.
- All 69 offline checks pass (68 in the ROS suite plus the added bridge regression).
- Headless Gazebo loads the tofu mesh; the imported mesh also appears in MoveIt,
  and arm/hand readiness plus five-stage replay preflight pass without motion.
- Simulation startup synchronizes generated models with gazebo2rviz's filesystem
  lookup. This prevents the legacy rect_object box from overriding mesh collisions.

## Optional TXT capture mode — 2026-11-10

- `grasp.mode: graspit` resolves the selected project-local capture on config
  load; `manual` (also the default when omitted) retains the YAML targets.
- All 76 offline tests pass, covering file selection, malformed/missing files,
  coupled joints, six-decimal limit rounding, object-relative frame conversion,
  configurable tightening and resolved recording snapshots.
- No additional motion execution was needed for this parser/config change.

## Thumb-first approach — 2026-11-10

- Diagnosed the stage-1 timeout in the running simulation: the arm IK exists,
  but the unprepared thumb intersects tofu (`rh_thhub` / `rh_thmiddle`).
- Stage 1 now prepares only `rh_THJ5` to the saved grasp target before moving
  the arm. Other hand joints hold their current angles. Stage 2 applies the
  remaining grasp angles. Both simulation and real replay follow this order.
- Read-only MoveIt checks in the running simulation successfully planned the
  thumb-only preparation (43 points) and the exact pose-based arm approach
  (12 points), with collision checking enabled. Neither path was executed.
- All 79 offline tests pass, including sequence order, replay's live finger
  holds, stopping before arm motion on preparation failure, and fast contact
  reporting when the approach is blocked.
- New recordings use task version 3; older runs are preserved and skipped.

## Simulation shutdown availability — 2026-11-10

- Simulation shutdown stays enabled during startup, active tasks, failed tasks,
  terminal prompts, and when the panel did not start the existing container.
- Task-terminal waiting observes shutdown cancellation and releases the session
  lock, allowing the normal container stop to terminate simulation processes.
- Other actions remain disabled while shutdown runs; duplicate shutdown clicks
  are ignored. Real hardware retains its existing calibration shutdown flow.
- All 22 desktop workflow tests pass, including cancellation of a waiting task
  while holding the session lock and cleanup after terminal launch failure.

## Reset simulation and independent offsets — 2026-11-10

- The simulation panel offers Reset simulation, which stops and recreates its
  session, reloading config, arm home, initial hand posture, world models and
  controller state. Recordings are preserved. It waits for readiness and
  restores the configured collision rule before enabling tasks.
- Shutdown remains available during reset; cancellation during its stop phase
  prevents the session from restarting. Reset is unavailable during an active
  task or in real hardware mode.
- `object.grasp_z_offset` defaults to 0.02 m and shifts only the object and
  newly planned grasp target in world Z. `robot.initial_z` is the single robot
  mounting height (currently 0.012 m) and changes only robot placement through
  the launch arguments.
- Offset checks cover both grasp sources, generated world geometry, unchanged
  floor/home settings and resolved recording snapshots.
- A direct in-place Gazebo joint reset was investigated but was not retained:
  hand/controller state and tracking do not reliably reset that way. Recreating
  the session restores the normal startup state. The existing simulated hand
  tracking limitation remains; numerical zero feedback is not guaranteed.
- All 89 offline tests pass, including reset orchestration/cancellation,
  independent base height, object/grasp offset and recording snapshots.

## Reset availability and saved-run dropdown — 2026-11-10

- Reset uses an independent interrupt path and stays enabled while startup or
  any task/terminal owns the session lock. It cancels the wait, stops the old
  container and starts a fresh session. Shutdown can cancel a pending reset.
- Queued work carries an operation generation so it cannot execute after the
  reset or shutdown that cancelled it; stale GUI callbacks cannot re-enable tasks.
- Both panels list saved runs by timestamp and status. Check and Replay pass
  the selected run explicitly; failed, incomplete or incompatible runs cannot
  execute. Listing does not deserialize pickle files.
- All 93 offline tests pass. A GTK test with a fake session verifies the actual
  Reset button stays enabled during busy/not-ready/task-active states, queued
  work is cancelled, and dropdown selection reaches the replay/check handler.

## Closed-hand vibration during arm motion — 2026-11-10

- Confirmed the loaded Shadow architecture: the trajectory controller supplies
  position demands to the per-joint Shadow PID controllers. Both layers are
  intentional, as documented in `SrGazeboHWSim::registerSecondHardwareInterface`.
- Compared two network-isolated Gazebo runs: 1 ms baseline and 0.25 ms physics,
  with original gains, the same captured closed hand, and wrist-1/wrist-2 sweeps
  of ±0.3/±0.2 rad over 3 seconds per sweep, away from the object.
- During arm motion, FFJ4 peak-to-peak motion dropped from 5.639° to 0.829°;
  RFJ4 from 7.115° to 0.002°; THJ4 from 3.218° to 0.924° in this comparison.
  After motion, FFJ4 dropped from 5.295° to 0.129°, RFJ4 from 9.629° to <0.001°.
- Reports are in `docs/measurements/hand_dynamics_1ms.json` and
  `hand_dynamics_0.25ms.json`. These are single-run comparisons, not a guarantee
  across all poses/contact loads. J1/J2 errors and load-dependent redistribution
  remain: Shadow's J0 controller controls the sum of those two joint angles.
- The configured simulation step is now 0.25 ms, with a matching requested
  4000 Hz physics update rate. More CPU may be required to maintain real time.
  No gains, object collision settings or real hardware were changed.

## JSON capture and gravity-aligning lift — 2026-11-10

- Selected the provided `grasps.json`, record 0, schema version 1. JSON import
  uses actual joint_value_rad (excluding fixed DH offsets), both captured
  world poses, and gravity_direction_world. The gravity vector is stored in
  rh_palm coordinates so rebasing the capture/object is consistent.
- The saved vector is [0.7122151531, -0.5445403550, -0.4429733372] in GraspIt
  world. In the configured ROS scene it initially points approximately
  [0.7122151531, 0.4429733372, -0.5445403550], requiring about 57° alignment.
- Stage 4 samples a vertical palm path and interpolated shortest rotation,
  converting every waypoint through the fixed flange/palm mount. Partial
  alignment paths are rejected. Preflight checks the reoriented endpoint IK.
- An isolated MoveIt test planned all 40 requested palm waypoints with collision
  checks, producing 81 joint points and 100% path completion in the configured
  open world. FK verified final palm [0.3567097526, -0.0467544691, 0.2953421994] m
  (20 cm rise with fixed X/Y) and gravity [2.0e-9, 1.4e-9, -1.0]. No motion or
  hardware execution was performed.
- Adding all configured real-scene cell walls interrupted the tested IK branch
  at side_plane_left. The verified result applies to the selected open world;
  other scenes/arm configurations still require successful collision checking.
- All 102 offline tests pass, including JSON joint offsets, transformed capture
  frames, antiparallel alignment, fixed palm translation, angular sampling,
  partial-path rejection, rotated IK endpoints and recording metadata.
- New recordings save alignment metadata and the rotated arm trajectory.
  Enter-per-stage replay and earlier recordings retain their existing behavior.

### Two-joint thumb preparation and faster lift (2026-11-10)

- New plans prepare `rh_THJ5` and `rh_THJ4` before the arm approaches, holding
  all other live hand angles. Replay uses the recorded preparation-joint list;
  earlier recordings without it retain their original one-joint preparation.
- Lift-only velocity and acceleration scaling are now 0.10, with a 16-second
  minimum duration (previously 20 seconds). Other arm stages remain at 0.08.
- All 104 offline tests pass, covering both preparation joints, live-angle
  preservation, legacy replay, recorded metadata, and lift retiming limits.
  No simulation or hardware motion was commanded for this change.
