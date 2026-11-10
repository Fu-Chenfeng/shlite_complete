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
