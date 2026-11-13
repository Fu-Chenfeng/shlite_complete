# Five-stage recordings and replay

GraspIt planning runs use the input filename without its extension:
`grasps.json` saves to `trajectories/grasps/`. Repeated saves use `grasps_001`,
`grasps_002`, etc., preserving earlier runs. Manual planning uses
`YYYY-MM-DD_HH-MM-SS` in Asia/Tokyo time. Both simulation and real control use
the same recording. Latest selection and the dropdown use the recorded creation
time, not alphabetical folder order.

```text
trajectories/<grasp-name-or-timestamp>/
  arm/
    Stage_1_Approach_grasp.pkl
    Stage_4_Lift.pkl
    Stage_5_Return_home.pkl
  hand/
    Stage_1_Prepare_thumb_base.pkl
    Stage_1_Prepare_thumb_base_target_joints.pkl
    Stage_2_Close_fingers.pkl
    Stage_2_Close_fingers_target_joints.pkl
    Stage_3_Tighten_grasp.pkl
    Stage_3_Tighten_grasp_target_joints.pkl
    Stage_5_Open_hand.pkl
    Stage_5_Open_hand_target_joints.pkl
  task.json
  run.json
```

`run.json` records the Tokyo start time, status and `task_version: 3`. A run
becomes complete after every stage succeeds; failed/interrupted runs stay
archived and are skipped by default replay. `task.json` records the five-stage
sequence, grasp pose and hand targets, arm home and planning settings.

| Stage | Behavior | Config input |
|---|---|---|
| 1 | Fully prepare thumb-base and finger J4 joints, partially close other joints, then approach | `grasp.joint_angles`, `planning.approach_hand_fraction`, palm pose |
| 2 | Fingers close to the grasp angles | `grasp.joint_angles` |
| 3 | Apply the second grasp configuration | `grasp.tightened_joint_angles` |
| 4 | Palm lifts in world Z and optionally aligns captured gravity downward | `lift_distance`, `lift_align_gravity` |
| 5 | Arm returns home, then hand goes to all zero | `robot.home_joints`; all active hand joints zero |

There is one replay prompt per stage. Stage 1 fully prepares `rh_THJ5`, `rh_THJ4`,
and each finger's J4, and moves other joints 60% from current angles toward their
targets, then moves the arm.
New recordings store `hand_preparation_mode: saved_target` and replay the complete
saved preparation target; older runs retain their original live-angle preparation.
Preparation failure stops before
arm motion. Stage 5 contains two actions; failure of
the home motion prevents the hand reset. A failed action stops the sequence.
New runs record `thumb_preparation_joints` in `task.json`; older runs without
that field keep their original `rh_THJ5`-only preparation.

The configured pose describes `rh_palm`, not the arm flange. Planning converts
the world-frame palm pose through the model's fixed mounting transform. The
lift keeps palm horizontal position and optionally interpolates its orientation
to align the saved palm-frame gravity vector with world -Z. Alignment requires
100% completion; fixed-orientation lifts use the configured path fraction.
Lift timing follows `lift_velocity`/`lift_acceleration`, falling back to the
arm scaling when omitted, and `lift_min_duration`. An unreachable configured path fails rather than saving a shorter lift.

Arm pickle files retain the standard ROS `RobotTrajectory` format and `ra_`
joint names. Replay checks the start and, when needed, plans to that first point
before executing with the saved timing on real hardware. Simulation replay uses
`simulation.replay_speed: 3.0` by default: motion durations are divided by three,
velocities multiplied by three, and accelerations multiplied by nine. This
applies to saved arm paths, the planned move to their start, and replanned hand
motions. Set it to `1.0` for normal speed. Saved files, planning speeds, Enter
prompts and settling pauses are unchanged. The speed override requires both a
simulation session and ROS simulation time; real replay stays at 1×.
The UR5 adapter still translates joint
names and goals to the existing driver for real control.
Equivalent 2π arm-joint representations are aligned to the current feedback
when the entire shifted path remains inside joint limits. This avoids an
unnecessary full wrist revolution when returning to the same home pose.

Hand stages replan to the **absolute joint maps saved with the run**. They do
not apply scaled deltas to current angles. All targets are checked against the
active group and joint limits before any stage executes. Preflight checks all
seven trajectories and four hand target maps, including the final arm home and
all-zero hand target. `--check` sends no motion or collision updates.
Hand trajectory starts are synchronized from fresh `/joint_states` feedback;
missing complete hand feedback stops execution. Existing start-state and
controller tolerances remain in effect.

Simulation replay temporarily disables all MoveIt collision checking.
Real replay keeps robot-table collision checking enabled in all stages,
including the move to the first saved arm pose and replanned hand motions.
Saved arm paths are checked against the table before execution, including
samples between saved points. A missing table model blocks real replay.
Other pairs remain disabled during replay. On completion, quitting or failure,
previous scene rules are restored while retaining real robot-table checking.
Planning retains its collision checks.

`object.hand_collision` is applied during planning/replay and restored afterward.
With `false`, it permits only hand-object contacts; other collision pairs stay
checked. Gazebo's physical contact filter follows the same setting after a
simulation restart. No stage deletes the object or relaxes all world collisions.

`trajectories.replay_run: latest` selects the newest completed version-3 run.
A timestamp can pin a compatible recording. `--trajectory-root` also accepts an
archive root or a specific run directory. The selected folder and saved lift
distance are printed before motion.

The 13 original files in `2026-11-10_13-35-00/` are preserved unchanged as a legacy
archive. They contain the old stage order and shorter lift. The five-stage
replay rejects them with a request to generate a new run; it never reinterprets
those old paths as the new task. Only load trusted pickle files.

Version-2 runs lack the initial thumb preparation. They remain archived and
are not replayed by the new sequence; generate a new plan to record all actions.

The desktop **Saved trajectories** dropdown lists named and timestamped runs with their
status. Choose a completed compatible run before Check or Replay; the panel
passes that exact folder to replay without editing config.yaml. Failed and
older-sequence runs remain visible for reference. Refresh list updates the
choices immediately; the panel also refreshes automatically while idle.

Gravity-aligning runs keep the same five stages and file format/version.
`task.json` includes `lift_alignment` with the captured palm-frame gravity
vector, target world direction and target palm quaternion. Stage 4's arm pickle
contains the actual rotated trajectory. Replay uses those saved joint positions,
not the current capture file. Existing recordings continue to replay unchanged.
