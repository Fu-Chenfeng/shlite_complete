# GraspIt / ROS hand definition check — 2026-11-10

The current `grasp_planner/graspit/source/models/robots/shadowhand_motor/`
kinematic definitions agree with this project's Shadow Lite URDF and the raw
13-DOF posture in `261110.txt`.

The audit recomputed the full GraspIt DH chain and each distal-link Inventor
mesh transform, then compared those with ROS forward kinematics. This checks
fingertip rotations as well as joint origins. Maximum distal-frame translation
difference was 0.000325 mm, and rotation difference was 0.000135 degrees,
consistent with export/capture rounding. GraspIt's flattened rotation matrices
are column-major; the world conversion maps GraspIt +Y to ROS +Z.

| Capture DOFs | ROS joints | Rule |
|---|---|---|
| d0, d1, d2 | FFJ4, FFJ3, FFJ2 | Direct radians |
| d3, d4, d5 | MFJ4, MFJ3, MFJ2 | Direct radians |
| d6, d7, d8 | RFJ4, RFJ3, RFJ2 | Direct radians |
| Finger final joint | FFJ1, MFJ1, RFJ1 | 0.8 × corresponding J2 |
| d9, d10, d11, d12 | THJ5, THJ4, THJ2, THJ1 | Direct radians |

The XML's +90°, +180° and −90° terms are fixed DH frame offsets. They are
already represented by the ROS model frames and are not added to ROS joint
commands. The capture pose uses XYZ millimetres and quaternion WXYZ; the
project converts to metres, palm frame and quaternion XYZW.

The **legacy** writer in `dependencies/graspit/ui/graspCaptureDlg.cpp` has a
misleading `CAPTURED_DOF_JOINT_ANGLES` block: it extracts the DOF index, ignores
its coupling multiplier, reverses the joint order, and scales values by 0.999.
For example, d2=0.1875 gives true FFJ1=0.1500 rad, while that block prints
0.1873125. The project reads the raw first posture line and applies the actual
0.8 coupling, so it does not use that helper block. `FINAL_JOINT_ANGLES` in this
legacy text is an adjustment block, not the absolute posture.

The **current** capture writer under `grasp_planner/graspit/source/ui/`
reads actual joints directly. In its JSON, `joint_value_rad` includes coupling
and excludes fixed model offsets; `angle_rad` includes fixed DH offsets and
must not be copied directly into ROS joint commands.

## Stage 2 failure in the running simulation

Read-only `/check_state_validity` queries found the current hand state valid.
The requested captured grasp and tightened grasp are collision-invalid. There
were no joint-limit violations in the imported targets.

The running scene uses a collision **box** for the tofu (`collision_mesh: false`)
and has hand-object collision checking enabled. Captured grasp contact samples:

| Link | Object overlap reported by MoveIt |
|---|---:|
| rh_ffdistal | 0.031 mm |
| rh_mfdistal | 0.186 mm |
| rh_rfdistal | 0.032 mm |
| rh_thdistal | 0.654 mm |

The captured grasp intentionally contacts the object in GraspIt. MoveIt's
current collision shapes and collision policy reject that endpoint, preventing
stage 2 from planning. This audit did not alter the grasp, collision settings,
live scene, or robot positions. Collision geometry/contact handling needs to be
chosen explicitly for executing this captured contact grasp.
