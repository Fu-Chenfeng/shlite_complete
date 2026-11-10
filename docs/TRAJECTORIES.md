# Saved data and execution contract

Host directory: `trajectories/`. Container directory: `/project/trajectories`.
`TRAJECTORY_ROOT` is the same in both Compose modes. The shared replay program
loads every required file and validates its joints before executing any stage.

Each planning run creates a new folder using its start time in **Asia/Tokyo**:

```text
trajectories/
  2026-11-10_13-35-00/
    arm/
    hand/
    run.json
  YYYY-MM-DD_HH-MM-SS/
    arm/
    hand/
    run.json
```

The existing 13 files were moved unchanged into `2026-11-10_13-35-00/`.
All stages of a new plan share one timestamp folder; another plan creates a new
folder. If two plans start within the same second, a numeric suffix prevents
overwriting. `run.json` records the start time, timezone and recording status.
Runs become complete only after planning/execution finishes successfully.
Failed/interrupted runs remain on disk, and default replay skips them.

Both simulation and real replay select the newest completed run automatically.
Choose an older run by setting `trajectories.replay_run` in `config.yaml` to its
folder name. The replay log prints the selected folder before checking files.
The optional `--trajectory-root` argument also accepts a specific run directory
or an archive root. The file paths below are relative to the selected run.

| Order | Group | File or operation |
|---|---|---|
| 0 | hand | All active hand joints to zero |
| 1 | hand | `hand/Stage_1_Hand_thumb-only.pkl` |
| 2 | arm | `arm/Stage_2_Move_arm_to_captured_hand_pose.pkl` |
| 3 | hand | `hand/Stage_3_Move_all_fingers.pkl` |
| 4 | arm | `arm/Stage_4_Move_arm_with_palm_shift.pkl` |
| 5 | hand | `hand/Stage_4_Iterated_hand_joint_angle_changes.pkl` |
| 6 | arm | `arm/Stage_5a_Move_arm_to_additional_offset_pose.pkl` |
| 7 | arm | `arm/Stage_5_Straight-line_Z_lift.pkl` |
| 8 | arm | `arm/After_Stage_5_Planned_joint_move_to_home.pkl` |

The copied baseline trajectories are byte-for-byte copies of the working
project. Planning uses the original label-to-filename conversion and Python
pickle message format. It also saves `Stage_0_Open_hand.pkl`, hand target maps
(`*_target_joints.pkl`) and the planned stage-4 hand trajectory, as before.

Arm replay keeps its `ra_` names in the shared files. Real hardware uses
`app/ur5_adapter.py` to translate joint states, trajectory goals, tolerances and
feedback between the combined model and the driver's unprefixed six-joint names.
Saved timing is retained. Newly planned moves to a trajectory's start use
velocity/acceleration scaling `0.05`; start tolerance is `0.02 rad`.

Hand replay plans to saved trajectory endpoints with velocity scaling `0.6`,
acceleration scaling `0.3`, goal tolerances `0.01`, and a `0.3` multiplier for
saved stage-4 deltas applied to the current angles. It does not reproduce the
hand's original recorded timing. The first replay stage zeros all active hand
joints, matching the working hardware UI's ZERO operation.

The planner retains the previous planning behavior in `app/plan.py`, with its
inputs now in `config.yaml` under `planning` and `robot.home_joints`,
including its separate `STAGE4_CHANGED_ANGLE_SCALE=0.5`. Therefore the initial
plan-and-save run and subsequent replay can differ in that hand adjustment.
Both **replay modes** use the same `0.3` setting and sequence. The planning command
executes only in simulation. It can temporarily relax environment checking under
`planning.stage4_ignore_env`, while respecting `object.hand_collision`.
It is not a real-hardware planning runner.

Planning and replay apply `object.hand_collision` to the `rh_` hand links and
the grasp object `rect_object`. With `false`, MoveIt allows that pair; with
`true`, it checks it. The original collision matrix is restored after the task.
This switch leaves other robot/object pairs unchanged. The planner keeps the
object present during Stage 5a rather than deleting its geometry. Gazebo's
physical hand-object filter follows the same setting after simulation restart.
The read-only `check` command does not change collision rules.

Older runs are preserved automatically. Do not replace files during replay.
Load only trusted pickle files. Hand controller
errors, failed arm goals and cancellation failures stop execution before the
next stage; the physical robot stop controls remain available when needed.
