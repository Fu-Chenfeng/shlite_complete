# Exact five-view camera mount — 2026-11-17

The original **0.4 m** orbit, 30° elevation, five azimuths, optical-center
offset and requested **180° image roll** are preserved. The five optical
target matrices are identical before and after changing the mount.

The assumed camera optical-center translation relative to `ra_flange` is now
`[-0.02, -0.08, 0]` m. Compared with `[0, -0.06, 0]`, this moves the camera
2 cm toward the wrist and 2 cm farther sideways. Its mounting orientation
stays at forward `[0.9949708551, 0, 0.1001648512]`, up `[0, 0, 1]`.

An isolated MoveIt check found all eight segments: close fist, start at arm
home, visit all five views, and return home. Hand-object, table and self
collision checking were enabled. Motion-rejecting dummy controllers received
no execution requests. No motion or image capture was executed.

The camera planner's arm joint goal tolerance is now `1e-5` rad. Forward
kinematics of the planned endpoints, composed with the camera mount, gives
maximum optical-center error **0.00747 mm** and orientation error
**0.00001896 rad** relative to the exact requested optical poses. These are
model endpoint errors; they do not measure execution tracking or calibration.

A separate center-ray check found no intersection with the modeled hand at
any view while holding the configured fist. It checks the center sightline,
not the full image, arm occlusion, or an actual camera/bracket/cable assembly.
This remains an assumed simulation mount awaiting physical calibration.

The yellow cone tips use the exact optical target positions, independently
of the selected arm mount.

Machine-readable settings, optical targets, planned joint targets, endpoint
errors and sightline evidence: [wrist_camera_exact_mount.json](wrist_camera_exact_mount.json).
