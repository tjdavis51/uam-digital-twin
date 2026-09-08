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
- This repository: project-owned scripts, documentation, and future ROS code.

## Direction convention

- `/fmu/out/*`: data published by PX4 for ROS 2 consumers.
- `/fmu/in/*`: data accepted by PX4 from ROS 2 publishers.

Future project-owned code will live under `src/`. The first planned package is
a C++ state gateway that converts PX4 NED/FRD conventions into standard ROS
ENU/FLU odometry and TF.
