# shlite_complete

UR5 + four-finger Shadow Hand Lite (hand_g G1M5): Gazebo simulation, simulation
planning/saving, real control through the existing hand NUC, and one shared replay
workflow. Linux with Docker Engine, Docker Compose v2 and an X11/XWayland desktop.

## First setup

```bash
cd ~/shlite_complete
./robotctl init
# Review .env, especially NUC_IP, PC_IP, UR5_IP and SSH_KEY.
./robotctl gui
./robotctl verify-images
```

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

Planning inputs are at the top of `app/plan.py`. Saving overwrites the matching
stage files in `trajectories/`; copy that folder first if keeping an older run.
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

Connect the laptop to the NUC network and the UR5 network. Check `.env` and NUC
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
| `compose.yaml`, `.env.example`, `robotctl` | Compose services and host commands |
| `app/` | Planner, shared replay, controller adapter and diagnostics |
| `launch/` | Simulation, combined MoveIt and original UR driver launch |
| `assets/`, `config/` | Worlds, models and robot/controller configuration |
| `trajectories/arm`, `trajectories/hand` | Shared saved files, included in transfers |
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
