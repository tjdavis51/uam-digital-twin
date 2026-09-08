# Checkpoint 01: PX4–Gazebo–ROS 2 Baseline

Captured 2026-09-08 from the working UTM VM `uam-digital-twin`.

## Host

```text
Ubuntu:       24.04.4 LTS
Architecture: aarch64
Kernel:       7.0.0-30-generic
CPU:          8 virtual cores
Memory:       15 GiB
Disk:         125 GiB, 98 GiB available at capture
```

## Core software

```text
ROS 2:                    Jazzy
Gazebo Sim:               8.11.0
PX4:                      v1.17.0
PX4 commit:               d6f12ad1c4f70ad3230afd7d86e971421e02fef4
px4_msgs:                 v1.17.0
px4_msgs commit:          86d8239e962f6939e05c3737784f60c02fa884db
Micro XRCE-DDS Agent:     v2.4.3
Agent commit:             73622810d984349b80bbac0ef55fc0b694d62222
GCC:                      13.3.0
CMake:                    3.28.3
Git:                      2.43.0
```

All three third-party repositories were clean when this checkpoint was
captured.

## Relevant installed packages

```text
ros-jazzy-desktop              0.11.0-1noble.20260615.092556
ros-jazzy-ros-gz               1.0.22-1noble.20260615.095917
ros-jazzy-gz-tools-vendor      0.0.7-1noble.20260225.130014
ros-jazzy-gz-sim-vendor        0.0.10-1noble.20260605.052554
python3-colcon-common-extensions 0.3.0-100
```

Exact apt patch versions are evidence, not hard pins. A fresh installation
uses the currently available Jazzy packages for Ubuntu 24.04; source revisions
that define PX4 message compatibility are pinned exactly.

## Validated behavior

- Gazebo X500 and PX4 SITL ran at approximately real-time speed.
- Ogre 1 rendered successfully through the UTM virtual GPU.
- PX4 connected to the Agent on `127.0.0.1:8888`.
- DDS time synchronization converged.
- ROS 2 discovered `/fmu/in/*` and `/fmu/out/*` topics.
- `sensor_combined`, `vehicle_attitude`, and `vehicle_odometry` delivered at
  approximately 100 Hz.
- `vehicle_local_position_v1` delivered at approximately 50 Hz.
- A 103.88-second MCAP baseline recorded 36,668 messages in 5.1 MiB.

The reference recording remains outside Git at:

```text
~/bags/px4_x500_baseline_2026-09-01
```

The repository's `validate_static.sh`, `run_dds_agent.sh`,
`run_px4_sitl.sh` (headless), and `validate_live.sh` scripts were also run
successfully against this checkpoint on 2026-09-08. A fresh-machine execution
of `bootstrap_ubuntu.sh` remains an acceptance test for the next clean Ubuntu
24.04 VM or workstation.

## Known non-blocking limitations

- Ogre 2 is not usable through the current UTM OpenGL 2.1 virtual GPU.
- The launch uses Ogre 1 by default.
- `libGstCameraSystem.so` is absent; it is not needed for the camera-free X500.
- A ground-control station was not running, so PX4 reported no GCS connection.
- The stock X500 is a quadrotor and is only an infrastructure test vehicle.
