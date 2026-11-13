# Captured GraspIt grasp 261110

Imported from `/home/chenfeng/DefMapSim/261110.txt`, using the hand and object
definitions in `DefMapSim/grasp_planner/graspit/source/models`.

The capture's first line contains 13 DOFs in radians. Its second line is the
object-relative hand-base position in millimetres and quaternion **w,x,y,z**.
This base is different from ROS `rh_palm`. The four fixed chain roots determine
the rigid palm transform; all 16 captured link origins independently agree
within **0.00024 mm** (capture precision and robot XML rounding).

Finger mapping: J4, J3, J2 receive each chain's three DOFs, and J1 receives
**0.8 × J2**, matching the robot XML. Thumb J5,J4,J2,J1 receive DOFs 9–12.
The legacy `CAPTURED_DOF_JOINT_ANGLES` helper ignores coupling and multiplies
DOFs by 0.999, so it is not the actual displayed posture. The all-zero
`FINAL_JOINT_ANGLES` block contains adjustment deltas. Two six-decimal source
values exceed exact URDF joint limits by less than one microradian; only those
rounding differences were clamped. Stage 3 retains the configured +10° tips.

Both object and hand use the same rigid rebase: **(x,y,z) → (x,-z,y)**, then
translation `[0.41, 0, 0.034472]` metres. GraspIt +Y becomes ROS +Z. The tofu
rests on the ground, preserving its original asymmetric mesh origin and the
captured relative grasp. The capture's object placement is consistent with
identity: its object-relative hand-base translation matches the exported world
base position, and the full hand registration agrees with the source posture.

`alignment.json` records transforms, mapping, bounds and numerical checks.
`verification.json` checks the actual configured posture against ROS URDF FK.
`preview.png` renders the project hand meshes and imported tofu at that pose;
the viewpoint differs slightly from the supplied screenshot.

The original object XML and WRL, plus an equivalent binary STL with unchanged
axes and millimetre coordinates, live in `../../models/tofuprintfinal/`.
The loose `/home/chenfeng/tofuprintfinal.stl` uses different axes and was not used.
The object retains the source 500 g mass; it remains static as configured.

This import does not retarget existing recordings. Generate a new plan before
replay. Numerical alignment and endpoint IK do not establish dynamic grasp
stability or successful real hardware execution.
