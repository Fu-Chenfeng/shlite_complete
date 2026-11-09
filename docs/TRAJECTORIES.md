# Saved data and execution contract

Host directory: `trajectories/`. Container directory: `/project/trajectories`.
`TRAJECTORY_ROOT` is the same in both Compose modes. The shared replay program
loads every required file and validates its joints before executing any stage.

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

The planner retains the previous inputs and planning behavior in `app/plan.py`,
including its separate `STAGE4_CHANGED_ANGLE_SCALE=0.5`. Therefore the initial
plan-and-save run and subsequent replay can differ in that hand adjustment.
Both **replay modes** use the same `0.3` setting and sequence. The planning command
executes only in simulation because it uses Gazebo services and temporarily
changes simulation collision handling. It is not a real-hardware planning runner.

During simulated replay, contact between the robot and the grasp object
`rect_object` is temporarily allowed in MoveIt so finger closure is plannable.
The original collision matrix is restored afterward. Other scene objects and
the real-hardware collision configuration are retained. The read-only `check`
command does not apply this contact allowance.

To keep multiple runs, copy `trajectories/` before generating another. Do not
replace files during replay. Load only trusted pickle files. Hand controller
errors, failed arm goals and cancellation failures stop execution before the
next stage; the physical robot stop controls remain available when needed.
