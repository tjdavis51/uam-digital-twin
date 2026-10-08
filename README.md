# UAM Digital Twin

Reproducible development environment and project-owned software for the
Unmanned Aerial Manipulator (UAM) digital twin.

This repository records the first validated infrastructure checkpoint:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8
- PX4 SITL v1.17.0
- `px4_msgs` v1.17.0
- Micro XRCE-DDS Agent v2.4.3
- X500 simulation with ROS 2 telemetry over UDP port 8888

Third-party repositories and generated build products are intentionally not
vendored here. Their exact revisions are recorded in `config/versions.env` and
`setup/workspace.repos`.

The dependency workspace uses normal `colcon build`, matching the validated
checkpoint. This repository is a separate project overlay; no nested ROS workspace
is needed. The existing dependency build remains unchanged.

## Controller workbench

```bash
./scripts/test_workbench.sh
```

The ROS-independent C++17 core builds on macOS or Linux with CMake. Infrastructure
tests pass; three algorithm acceptance tests intentionally skip until you implement
the PD, hold, and smooth-axis functions. Use `-DUAM_REQUIRE_ALGORITHMS=ON` to make
unfinished algorithms fail acceptance. See [the implementation guide](docs/WORKBENCH.md)
and [interface conventions](docs/INTERFACES.md).

On the Ubuntu/Jazzy VM, `./scripts/build_project.sh` builds `uam_core` and
`uam_interfaces` into `install/project`; the existing `build_all.sh` still builds
only third-party support and PX4.

The physical UAM is a primary target. Gazebo X500 is a temporary PX4 integration
and safety fixture, not a representative UAM modeling target. Physical auditing
and baseline collection can proceed alongside SITL work after numerical acceptance.
Begin Simcenter restoration when access is available; the intended long-term
simulator remains PX4 SITL + Simcenter, subject to feasibility.

## Existing machine quick start

Open three terminals.

Terminal 1:

```bash
./scripts/run_dds_agent.sh
```

Terminal 2:

```bash
./scripts/run_px4_sitl.sh
```

Terminal 3:

```bash
./scripts/validate_live.sh
```

The Gazebo launch defaults to Ogre 1 for compatibility with the UTM virtual
GPU. Override it on a native Linux workstation with, for example:

```bash
UAM_RENDER_ENGINE=ogre2 ./scripts/run_px4_sitl.sh
```

## Fresh Ubuntu 24.04 workstation

```bash
git clone <repository-url> UAM_Digital_Twin
cd UAM_Digital_Twin
./setup/bootstrap_ubuntu.sh
```

Log out and back in if the bootstrap script changes group membership, then:

```bash
./scripts/build_all.sh
./scripts/validate_static.sh
```

The bootstrap script is deliberately conservative: if an existing third-party
checkout is at a different commit, it stops instead of overwriting work.

## Default filesystem layout

```text
~/dev/PX4-Autopilot       pinned PX4 source and SITL build
~/ros2_px4_ws             px4_msgs and Micro XRCE-DDS Agent workspace
~/venvs/px4               PX4 Python environment
~/bags                    local recordings (not committed)
```

All locations can be overridden through environment variables documented in
`config/versions.env`.

## Documentation

- `docs/ARCHITECTURE.md`: current process and data-flow architecture
- `docs/CHECKPOINT_01.md`: evidence captured from the validated VM
- `setup/workspace.repos`: exact ROS-side source revisions
- `config/px4-venv-requirements.lock`: Python environment snapshot

## Data policy

Rosbags, PX4 logs, compiled binaries, and VM images do not belong in ordinary
Git history. Store selected datasets in an external data archive or a
Git-LFS/DVC-backed repository when the data-management plan is established.
