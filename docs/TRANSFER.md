# Transfer shlite_complete to another PC

Use a Linux x86_64 PC with Docker Engine and Compose v2. GUI use needs an X11
or XWayland session and the host `xhost` tool (`x11-xserver-utils` on Ubuntu).
SSH, `ip`, NetworkManager and Chrony are host tools. This project does not install
Docker, graphics drivers, network configuration or NUC credentials automatically.

## Recommended: transfer the exact working images

On this working laptop, from the project directory:

```bash
./robotctl verify-images
./robotctl export-images /path/on/USB/shlite_complete-images.tar
```

This archive contains the Shadow workstation image and the UR5 image; it can be
large. Use a drive/filesystem with enough space and support for files over 4 GB.
Stop planning/saving before copying project data, then package the clean folder:

```bash
tar --exclude=shlite_complete/.env --exclude='*/__pycache__' \
  -czf /path/on/USB/shlite_complete.tar.gz -C "$HOME" shlite_complete
cd /path/on/USB
sha256sum shlite_complete.tar.gz shlite_complete-images.tar > SHA256SUMS
```

On the **new PC**:

```bash
cd /path/on/USB
sha256sum -c SHA256SUMS
# Ensure ~/shlite_complete does not already contain another installation.
tar -xzf shlite_complete.tar.gz -C "$HOME"
cd ~/shlite_complete
./robotctl init
./robotctl import-images /path/on/USB/shlite_complete-images.tar
./robotctl verify-images
./robotctl gui
```

Extract onto the Linux filesystem rather than running directly from an NTFS USB.
`.env` is generated for the new user's UID/GID and display. Review its addresses;
authorize the new laptop using [NEW_MACHINE.md](NEW_MACHINE.md). SSH keys are not
included in the project or Docker image archive.

## Online alternative

The Compose file can pull the Shadow image and build the UR5 image from the
included `docker/ur5.Dockerfile`:

```bash
./robotctl init
docker compose --profile sim pull sim
./robotctl build
```

Building requires access to the ROS/Ubuntu package repositories. Freshly pulled
or rebuilt images may differ from `images.lock.json`, which records the images
used for this migration. Import the exported archive to reproduce them exactly.
The NUC's existing image/container and calibration remain on the NUC and are not
part of this workstation image archive.

## What's included

Project code, the UR5 Dockerfile and model overlay, simulation worlds/models,
hand controller configuration, and all current `trajectories/arm` and
`trajectories/hand` files travel together. No dependency on the old project
directories or the USB backup remains. The two runtime modes use automatically
named Compose containers under project `shlite_complete`, avoiding the former
`ur5-noetic` container-name conflict. Stop old ROS sessions before starting this
project: host networking still shares the ROS port and robot connections.
