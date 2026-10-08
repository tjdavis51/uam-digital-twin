# Current Architecture

```text
Gazebo Harmonic
  physics, X500 model, simulated sensors
          |
          | sensor data / actuator commands through PX4 gz_bridge
          v
PX4 v1.17.0 SITL
  uORB, EKF2, commander, flight-control loops
          |
          | uXRCE-DDS client over UDP 8888
          v
Micro XRCE-DDS Agent v2.4.3
          |
          | DDS
          v
ROS 2 Jazzy
  px4_msgs, /fmu/in/*, /fmu/out/*, rosbag2
```

Gazebo owns simulated physical truth. PX4 receives simulated sensor data and
produces an estimated vehicle state. `/fmu/out/vehicle_odometry` is therefore a
PX4 estimator output, not simply a copy of perfect Gazebo ground truth.

The current stock X500 validates the infrastructure. It does not yet contain
the hexacopter geometry, manipulator joints, contact model, mocap interface, or
project-owned flight/docking controller.

## Environment layers

- UTM VM: complete Ubuntu ARM64 computer.
- `/opt/ros/jazzy`: system ROS 2 underlay and Gazebo vendor packages.
- `~/ros2_px4_ws`: ROS overlay containing `px4_msgs` and the DDS Agent.
- `~/venvs/px4`: Python dependencies used by PX4 development tools.
- `~/dev/PX4-Autopilot`: pinned third-party PX4 source and SITL build.
- This repository: project-owned scripts, documentation, standalone uam_core, and
  uam_interfaces ROS messages; built as a project overlay.

## Direction convention

- `/fmu/out/*`: data published by PX4 for ROS 2 consumers.
- `/fmu/in/*`: data accepted by PX4 from ROS 2 publishers.

Project-owned code lives under `src/`. The numerical workbench is implemented as
scaffolding; mathematical functions remain user-owned TODOs. See `WORKBENCH.md`
for build/test instructions and `INTERFACES.md` for the authoritative new project
contract. No state gateway or offboard command publisher is implemented yet.

## Research targets

Physical UAM development is primary. Gazebo X500 is a temporary integration and
safety fixture for PX4 interfaces, frames, timing, lifecycle and initial control;
do not invest in making it a representative UAM. Physical auditing/baseline data
collection may proceed alongside safe SITL validation once the core works.
Simcenter restoration begins when access is available, independently of Gazebo
polish. The intended long-term simulation path is PX4 SITL + Simcenter if feasible.
All project mathematical interfaces and experiment definitions remain independent
of the plant. No Siemens bridge is included in this milestone.
