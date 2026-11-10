# shlite_complete

UR5 + four-finger Shadow Hand Lite (hand_g G1M5): Gazebo simulation, simulation
planning/saving, real control through the existing hand NUC, and one shared replay
workflow. Linux with Docker Engine, Docker Compose v2 and an X11/XWayland desktop.

## First setup

The shared settings are in **[`config.yaml`](config.yaml)**: object model/mesh,
pose, dimensions, color, environment obstacles, robot startup/home pose, network
addresses and planning targets. `.env` contains Docker images, host UID/GID and
display settings. See [CONFIGURATION.md](docs/CONFIGURATION.md) for examples.

Set `object.hand_collision: false` to disable hand-object contacts in Gazebo and
hand-object collision checking in MoveIt; set `true` to enable them again.
Restart simulation after changing this setting.

```bash
cd ~/shlite_complete
./robotctl init
# Review config.yaml (especially network) and the generated .env.
./robotctl gui
./robotctl verify-images
```

In Ubuntu's **Files** app, open **Home → shlite_complete**, right-click
**`Simulation.sh`** and choose **Run as a Program**. For real control, do the same
with **`Real-Control.sh`**. These executable launchers ship inside the project and find `robotctl` beside them,
so they work after copying or moving the folder without installation or path
updates. Each opens a native control panel with
Start, task check, step-by-step Replay, Status, Edit config.yaml and Shut down
buttons. Simulation also offers Plan, execute & save. No commands need to be
typed for everyday sessions. The optional `./robotctl shortcuts` command only
regenerates the launchers inside the project; it writes no external shortcuts.

Start real hardware launches ROS core, the NUC hand loop, UR5 driver, adapter and
MoveIt in order, waiting for readiness. Keep the hand clear during calibration;
start the External Control program on the pendant when prompted. Startup never
replays a task. Replay opens a terminal for Enter/q stage prompts. Finish or quit
the task and close its terminal before using Shut down. Keep the panel open
until shutdown completes; real shutdown retains the original hand calibration.
SSH access must already work (unlock an encrypted key in your SSH agent first).

Host desktop controls require `python3-yaml`, `python3-gi`, `gir1.2-gtk-3.0`,
`xdg-utils` and a terminal such as `gnome-terminal`. The command-line workflow
below is also available; it requires `python3-yaml` but does not require GTK.

This machine already has the two required images. For another PC, follow
[TRANSFER.md](docs/TRANSFER.md) and [NEW_MACHINE.md](docs/NEW_MACHINE.md).
`up` starts idle workstation containers; hardware starts only with the explicit
`real nuc` and `real ur5` commands. Stop the old project's sessions before use.
Legacy saved files are included unchanged; use your new trajectories for the
next replay. Validation scope is recorded in [VALIDATION.md](docs/VALIDATION.md).

## Simulation

**Terminal 1:**

```bash
cd ~/shlite_complete
./robotctl sim up
./robotctl sim start
```

Wait for Gazebo, RViz and MoveIt. **Terminal 2:**

```bash
cd ~/shlite_complete
./robotctl sim plan       # plan, save and execute the original task in Gazebo
```

Planning inputs are in `config.yaml` under `planning` and `robot.home_joints`.
Each planning run saves into a new `trajectories/YYYY-MM-DD_HH-MM-SS/` folder
(Tokyo time), with `arm/` and `hand/` inside. Earlier runs are preserved.
Replay uses the latest completed run by default; set `trajectories.replay_run`
in `config.yaml` to a timestamp folder name to select an older run.
To replay the currently saved task instead of generating another:

```bash
./robotctl sim check
./robotctl sim replay     # Enter before each stage; q quits between stages
# Or: ./robotctl sim run  # all stages, sequentially, without prompts
```

Shutdown, from another terminal:

```bash
./robotctl sim shutdown
```

The default world is `assets/worlds/open.world`, copied from your usual
`my_world_no_ob.world`. To use the cell-obstacle variant:
`./robotctl sim start world:=/project/assets/worlds/cell.world`.

## Real arm + hand

Connect the laptop to the NUC network and the UR5 network. Check `config.yaml` and NUC
clock synchronization as described in [NEW_MACHINE.md](docs/NEW_MACHINE.md).
Finish and stop the simulation first. Keep the hand clear during calibration.

**Terminal 1:**

```bash
cd ~/shlite_complete
./robotctl real up
./robotctl real core
```

Wait for `started core service [/rosout]`. In **separate terminals**, each from
`~/shlite_complete`, run in order:

```bash
# Terminal 2: real hand hardware; wait for initialization
./robotctl real nuc

# Terminal 3: real UR5 driver; start the usual External Control program on the pendant
./robotctl real ur5

# Terminal 4: arm joint/action adapter
./robotctl real adapter

# Terminal 5: combined arm-and-hand MoveIt + RViz
./robotctl real moveit
```

Leave these terminals running. In **terminal 6**:

```bash
cd ~/shlite_complete
./robotctl real obstacles
./robotctl real check
./robotctl real status
```

Confirm RViz matches the physical assembly and the UR5's scaled trajectory
controller is `running`. Then replay:

```bash
./robotctl real replay    # Enter executes the next stage; q quits between stages
# Or: ./robotctl real run
```

Finish motion and quit replay. Keep the other terminals open, then run:

```bash
./robotctl real shutdown
```

This performs the original hand shutdown calibration, stops all ROS nodes on
this session's master, and stops the NUC and workstation containers. The laptop,
NUC and UR control box stay powered on. If calibration or SSH fails, the command
stops at that failure so you can inspect it. Do not close ROS core first.

## Shared trajectories and sequence

Both modes use **`app/replay.py`**, **`trajectories/`**, and the same sequence:

1. Zero/open hand.
2. Thumb-only hand goal.
3. Arm approach to captured hand pose.
4. Move all fingers.
5. Arm palm shift.
6. Hand adjustment using saved deltas.
7. Arm additional offset.
8. Straight Z lift.
9. Arm home.

Every stage waits for completion; a failed stage stops the sequence. Arm paths
are replayed with saved timing after checking/moving to their first point. Hand
paths are replanned to saved final joint goals; stage 6 applies saved deltas to
the current hand position. This preserves the previous working replay behavior.
See [TRAJECTORIES.md](docs/TRAJECTORIES.md) for files and settings.

## Project contents

| Path | Purpose |
|---|---|
| `config.yaml` | Object, scene, robot, network and planner definitions |
| `Simulation.sh`, `Real-Control.sh`, `app/control_panel.py` | Project launchers and button controls |
| `compose.yaml`, `.env.example`, `robotctl` | Compose services and host commands |
| `app/` | Planner, shared replay, controller adapter and diagnostics |
| `launch/` | Simulation, combined MoveIt and original UR driver launch |
| `assets/`, `config/` | Worlds, models and robot/controller configuration |
| `trajectories/<date_time>/arm`, `trajectories/<date_time>/hand` | Archived saved runs, included in transfers |
| `docker/` | Shadow startup and self-contained UR5 Dockerfile |
| `docs/` | New-machine registration, transfer and trajectory reference |
| `tests/`, `images.lock.json` | Offline validation and tested image identities |

No source directory outside this project is required. The existing NUC retains
its own `dexterous_hand` container, hand calibration and hardware configuration.
This project is a workstation transfer, not a NUC backup.

## License

Original project code and documentation are available under the [MIT license](LICENSE).
Upstream robot configuration and model files retain their original licenses;
see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
