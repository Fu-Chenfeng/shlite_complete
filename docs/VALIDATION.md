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
