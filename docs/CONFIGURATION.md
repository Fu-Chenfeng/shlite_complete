# Shared configuration and desktop controls

Edit `config.yaml` in the project root, or use **Edit config.yaml** in either
desktop panel. All distances are metres and angles are radians. Settings apply
to the existing `ur5` + right `hand_g` / `G1M5` assembly; changing robot type or
hand hardware still requires matching vendor launch and controller files.

## Object and environment

```yaml
object:
  enabled: true
  model: assets/models/rect_object/model.sdf
  pose: [0.41, 0, 0.0675, 0, 0, 0]   # x y z roll pitch yaw, world frame
  static: true
  mass: 0.2
  visual_size: [0.035, 0.09, 0.075]
  collision_size: [0.031, 0.087, 0.072]
  hand_collision: true               # false disables hand-object contacts
  color: [0.8, 0.2, 0.2, 1.0]        # red green blue alpha
  mesh: null                         # null uses the visual box
  mesh_scale: [1.0, 1.0, 1.0]
  add_to_real_scene: false
```

To display the included STL, set `mesh: assets/models/tofu_real.stl`. Set
`mesh_scale` to match the mesh's units; for a millimetre-based STL use
`[0.001, 0.001, 0.001]`. The independently configured `collision_size` remains a
box. Models and meshes must be inside the project so containers and transfers
can access them. `model` is an SDF template with the same single-link, box
collision/inertial/material structure as the supplied `rect_object/model.sdf`.
Its runtime name remains `rect_object` for compatibility with the task planner.

Set **`object.hand_collision: false`** to let the hand pass through the object in
Gazebo and to allow that same pair in MoveIt. Set it to `true` to check those
collisions again. Restart simulation after changing this setting: Gazebo loads
the physical filter with the world. The desktop Start button applies the
MoveIt rule, and each new plan/replay reads it again. Check-only replay remains
read-only. The switch targets the `rh_` hand links and the object; it keeps the
object's collision geometry and its Gazebo contacts with the arm and floor.
Other MoveIt pairs are unchanged by this switch; the existing
`planning.stage4_ignore_env` option separately relaxes environment checks during
its specified planning stages.

On real hardware this controls MoveIt checking when the object is present in
the planning scene (`add_to_real_scene: true`, or added externally). It cannot
disable physical contact forces on the real hand. Gazebo's filter is compiled
automatically inside the existing Shadow image; the host needs no compiler.
The filter uses Gazebo Classic's [collision bitmasks](https://get.gazebosim.org/tutorials?cat=physics&tut=collide_bitmask).

`simulation.world` selects `assets/worlds/open.world` or
`assets/worlds/cell.world`. The `environment` section sets obstacle sizes and
poses. Simulation updates obstacles included in the selected world; the open
world keeps only the ground, and the cell world includes the surrounding walls.
Real startup adds every enabled environment obstacle to MoveIt. The object's
collision box is added to real MoveIt only when `add_to_real_scene` is true.

Simulation startup writes a world and object SDF under `.runtime/`; the original
assets remain unchanged. The planner keeps the object in the scene throughout
the task and applies the configured hand collision rule. Restart after scene or robot placement
changes. `open_no_collision.world` is also accepted as a base world, but object
collision dimensions are always controlled by `object.collision_size`.

## Robot, network and planning

- `robot.initial_y` / `initial_z`: assembly placement in simulation and real MoveIt.
- `robot.home_joints`: simulation startup and the planner's final home target.
- `network`: workstation/NUC/UR5 addresses, SSH identity path and ROS port.
- `planning`: arm poses, hand joint targets/deltas, tolerances and lift settings.
- `trajectories.replay_run`: `latest` (newest completed run), or a specific
  `YYYY-MM-DD_HH-MM-SS` folder name from `trajectories/`.

Existing planner values are preserved as the defaults. Planning edits apply to
the next new plan. They do not retarget previously saved trajectories. Generate
new trajectories for a changed object/task before replaying that task.

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
be started with Enter, or the task quit with q between stages. Finish/quit the
task and close its terminal before shutdown. **Plan, execute & save** creates a
new timestamped folder with `arm/` and `hand/` subfolders, using Tokyo time.
Failed or interrupted planning runs remain archived; default replay skips them.
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
