# shlite_complete

A Docker-based workspace for a **UR5 arm with a four-finger Shadow Hand Lite**
(`hand_g`, `G1M5`). It provides Gazebo/RViz simulation, GraspIt-driven grasp
planning, saved trajectories, and replay on simulation or real hardware through
an existing hand-control NUC.

The five-stage task approaches the grasp, closes and tightens the fingers,
lifts with optional gravity alignment, then returns home. Project launchers
provide button controls for planning, replay, reset and shutdown.

## Installation

Use a Linux x86_64 desktop with X11 or XWayland. ROS runs inside Docker.

1. Install **Docker Engine and Compose v2** using the
   [Docker Ubuntu installation guide](https://docs.docker.com/engine/install/ubuntu/).
   Configure [Docker access for your login user](https://docs.docker.com/engine/install/linux-postinstall/)
   so `docker info` and `docker compose version` work without `sudo`.
2. Install the host dependencies:

   ```bash
   sudo apt update
   sudo apt install git python3 python3-yaml python3-gi gir1.2-gtk-3.0 \
     xdg-utils x11-xserver-utils gnome-terminal openssh-client
   ```

3. Clone and initialize:

   ```bash
   git clone https://github.com/Fu-Chenfeng/shlite_complete.git
   cd shlite_complete
   ./robotctl init
   ```

4. Get the runtime images:

   ```bash
   docker compose --profile sim pull sim
   ./robotctl build  # Build the UR5 image for real control; optional for simulation.
   ```

   To reproduce the tested images exactly, use the image archive procedure in
   [TRANSFER.md](docs/TRANSFER.md) instead. Fresh image builds may differ from
   `images.lock.json`.

5. Review [`config.yaml`](config.yaml) for the grasp, object, robot and network
   settings, and `.env` for Docker/display settings. Allow the GUI connection:

   ```bash
   ./robotctl gui
   ```

## Run

Open **`Simulation.sh`** or **`Real-Control.sh`** from the project folder
(right-click → **Run as a Program** in Ubuntu Files), or run:

```bash
./Simulation.sh
# For real hardware: ./Real-Control.sh
```

In simulation, click **Start**, then **Plan, execute & save**, or select a saved
trajectory and **Replay**. Replay waits for Enter before each stage; `q` quits.
Saved runs are included under [`trajectories/`](trajectories/).

For real hardware, first configure the NUC/UR5 connections and clock synchronization
using [NEW_MACHINE.md](docs/NEW_MACHINE.md) and the
[real-control workflow](contents.md#real-arm--hand). Stop simulation before
starting real control.

## Documentation

- [Detailed introduction, workflow and project contents](contents.md)
- [Configuration reference](docs/CONFIGURATION.md)
- [Trajectory format and replay behavior](docs/TRAJECTORIES.md)
- [Image transfer](docs/TRANSFER.md) · [Validation](docs/VALIDATION.md)

## License

Original code and documentation: [MIT](LICENSE).
Robot/model assets retain their upstream licenses; see
[third-party notices](THIRD_PARTY_NOTICES.md).
