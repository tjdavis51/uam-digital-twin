# Control and plant interface audit

Date: 2026-09-29. Static inspection of the Raspberry Pi backup, JJ's knowledge-transfer archive, and selected public repositories. No aircraft connection, arming, controller execution, MATLAB execution, or Siemens solver execution was performed. Source presence establishes a candidate capability, not a currently deployed or flight-validated capability.

## Findings that affect the architecture

1. The Pi backup contains `px4_msgs`, `px4_ros_com`, and Micro XRCE-DDS Agent. The visible ROS source and install spaces contain only the first two ROS packages. A complete deployed research controller and startup manifest were not found in this backup.
2. JJ's public `position_to_accel` repository contains a real cascaded P–PID controller and a disturbance-observer variant. Both publish acceleration setpoints and leave lower flight stabilization to PX4. They use legacy message fields/topics and cannot be copied into the current workspace unchanged.
3. The MECC archive contains multiple simulation controllers: an 18-PID aerial-manipulator model, inverse-dynamics controllers, operational-space controllers, and wrench observers. Their existence does not establish that the physical aircraft ran them.
4. A V004 Siemens interface is concretely recoverable: Simulink calls `vlmotionmex` with `antype='cosim'`. The inspected EKF/WO model accepts six body-wrench components and two servo torques. It is not a position/velocity-command plant.
5. The current `plantout.m` and saved EKF/WO SLX expose 53 outputs. An older spreadsheet lists 39 with a different order. Port order must be bound to the selected model, not inferred from that spreadsheet.
6. PX4 SITL against Siemens is a credible candidate, but no working PX4–Siemens sensor/actuator bridge was found in the inspected material. Existing code supplies useful pieces, not a completed integration.

## Raspberry Pi backup

Root: `../../backups/drone_backup/`.

| Evidence | Observed fact | Limit of the conclusion |
| --- | --- | --- |
| `ros2_ws/src/px4_msgs` Git HEAD | `d3673f41976f04c64a5e559a61a441f348501913`; commit subject references PX4 `3f9c6ec2c3f93d233b1a54a8d59ef055c7675b4c` | Message checkout does not establish aircraft firmware |
| Installed `px4_msgsConfig-version.cmake` | Package version `1.17.0` | Not proof of a matching running flight controller |
| `ros2_ws/src/px4_ros_com` Git HEAD | `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` | The backup mixes independently versioned repositories |
| `src/examples/offboard/offboard_control.cpp` | Publishes `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`; 100 ms timer; position mode and fixed position `{0,0,-5}` | Stock example, not the UAM feedback controller |
| `src/examples/offboard_py/offboard_control.py` | Subscribes to local position/status and publishes position setpoints | Example logic, not evidence of deployment |
| `TrajectorySetpoint.msg` | Position, velocity, acceleration arrays, yaw and yaw speed; NED convention; NaN means uncontrolled | Available message schema is broader than confirmed firmware topic exposure |
| `OffboardControlMode.msg` | Position, velocity, acceleration, attitude, body-rate, thrust/torque, direct-actuator flags | Flags do not prove each mode works on the aircraft |

The project's own [checkpoint](CHECKPOINT_01.md) is separate evidence: it records PX4 SITL v1.17.0 at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` and `px4_msgs` at `86d8239e962f6939e05c3737784f60c02fa884db`. Do not silently treat either checkout as the aircraft's firmware revision.

The scoped backup search did not recover a current FCU parameter export, firmware build manifest, current `dds_topics.yaml`, complete startup services, or a run manifest tying a controller revision to an aircraft flight. Those are the main remaining hardware evidence gaps.

## Recovered ROS controller chain

Public snapshots inspected:

| Repository | Snapshot prefix | Role |
| --- | --- | --- |
| [garrard_trajectorysetpoint](https://github.com/yzgarrard/garrard_trajectorysetpoint/tree/b013790) | `b013790` | Publishes a vertical sine position reference every 100 ms |
| [position_to_accel](https://github.com/yzgarrard/position_to_accel/tree/ddc3a93) | `ddc3a93` | Position P loop followed by velocity PID; optional disturbance observer |
| [px4_offboard_control](https://github.com/yzgarrard/px4_offboard_control/tree/2eb0414) | `2eb0414` | Offboard/arming and setpoint forwarding; also a separate waypoint example |
| [mocap_px4_bridge](https://github.com/yzgarrard/mocap_px4_bridge/tree/5c21bb7) | `5c21bb7` | PoseStamped to PX4 visual odometry |
| [fake_mocap](https://github.com/yzgarrard/fake_mocap/tree/b562bb6) | `b562bb6` | Synthetic mocap publisher candidate for interface tests |
| [Drone_Motor_RPM_Sensor](https://github.com/yzgarrard/Drone_Motor_RPM_Sensor/tree/616f54d) | `616f54d` | Motor-edge timing and Python I2C/CSV acquisition |

The legacy command path is:

```text
garrard_trajectorysetpoint_pub
    -> position_to_accel_ppid_node (50 Hz timer)
       feedback: VehicleLocalPosition_PubSubTopic
    -> garrard_accelerationsetpoint_pubsub
    -> offboard_control_node_external_traj_source
    -> TrajectorySetpoint_PubSubTopic
    -> PX4 acceleration-to-attitude/thrust and inner stabilization
```

The controller sets position and velocity fields to NaN, sets acceleration mode, and supplies acceleration plus yaw. P–PID means a proportional position loop generates desired velocity, and a proportional/integral/derivative velocity loop generates acceleration. This is a useful predecessor for our architecture, but its exact correspondence to a successful historical flight is unverified.

### Reuse issues found by source inspection

In [position_to_accel_ppid_node.cpp](https://github.com/yzgarrard/position_to_accel/blob/ddc3a93/src/position_to_accel_ppid_node.cpp):

- Line 57 schedules every 20 ms, while lines 88–89 and 115 clamp the computed state timestamp interval to approximately 6.94–10.42 ms. That discrepancy changes integral/filter behavior when updates arrive at the scheduled interval.
- Lines 185–187 calculate a velocity difference multiplied by `dt`, rather than divided by `dt`, despite using it as derivative action. This needs reconciliation with the intended discretization and flight-tested revision.
- Acceleration magnitude checks are used in integrator logic, but there is no clear final hard clamp on the published acceleration after all contributions. A configured maximum is not necessarily an enforced command limit.
- Initialization, stale-state handling, authority transitions, and command expiration need explicit review.
- Old scalar trajectory fields (`x`, `vx`, etc.) and `Timesync_PubSubTopic` do not match the array-based message contract in the inspected Pi checkout.

The external forwarding node also sets some timestamps to zero and requests arming after a timer count. Both it and the controller publish offboard mode. Our implementation should have one authority owner and explicit lifecycle handling. These are findings about recovered source, not a claim that JJ flew this exact snapshot unchanged.

## Motion capture path

The inspected bridge subscribes to `/Robot_1/pose` (`geometry_msgs/PoseStamped`) and publishes `/fmu/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry`). Its README assumes local FLU world/body frames, including Z-up in Motive. The code flips Y/Z signs, reorders the quaternion into scalar-first form with the corresponding sign changes, and labels the result `POSE_FRAME_FRD`.

This is a local FLU-to-FRD convention, not a general ENU-to-NED conversion. Our frame contract must distinguish the laboratory world orientation from geographic ENU/NED. The bridge forwards the incoming source timestamp in microseconds and sets `timestamp_sample` equal to it; it does not itself establish clock synchronization.

Only pose is explicitly populated. Velocity, covariance/unknown-value semantics, quality, sample timing, marker-to-body offset, and estimator fusion settings need checking against the deployed firmware. The archived workshop guide suggests external-vision position/yaw fusion, but it is an example configuration, not the aircraft's parameter export.

For comparison experiments, preserve three distinct streams: raw mocap, PX4's fused state, and simulator truth. Sending mocap into PX4 does not make the controller's fused state identical to raw mocap.

## MATLAB/Simulink controller inventory

143 SLX files were statically inventoried; 111 unique file hashes were found. XML blocks were inspected without loading or running the models. Detailed review focused on the following candidates; this was not a line-by-line review of every model.

| Candidate under `../../backups/NG_ASU_KT/` | What was found | Assessment |
| --- | --- | --- |
| `MECC_2023/Aerial_Manipulator/Sanity_check_AM_continuous.slx` | 18 PID blocks: 12 aircraft and 6 manipulator | Matches the April 25, 2022 report's preliminary Simcenter/Simulink PID demo description; not proof of physical peg insertion |
| `NGC/2022.05.09_Realize_Live/Sims/AIAA_2022_Final_Controller_2.slx` | Position/velocity/attitude/rate PID blocks; four motor-force outputs in ground-effect logic; `vlmotionmex` | Older quadrotor simulation baseline, not the current hexacopter controller |
| `MECC_2023/AerialManipulatorV2/JJ_20220921_cosim_motion_control.slx` | Inverse dynamics, trajectory generation, wrench estimation | More advanced simulation control than a PID-only design |
| `MECC_2023/AerialManipulatorV2/JJ_20220928_cosim_operational_space_control.slx` | Pose error through a Jacobian; mass/Coriolis/gravity compensation; desired tilt; rotor-force allocation | Candidate lineage for simulated peg insertion; not proven to be the exact demo revision |
| `MECC_2023/AerialManipulatorV003/Inverse_Dynamics_Controller.m` | Feedforward acceleration plus PD error, dynamics compensation, desired tilt and thrust-axis projection | Readable reusable mathematical reference for coupled control |
| `MECC_2023/AerialManipulatorV004_EKF_WO/UAM_check_model_Absolute_body_CSYS.slx` | Coupled inverse dynamics, state estimation/noise branches, wrench observers, Siemens plant | Research simulation baseline, not PX4-equivalent flight stabilization |
| `AIAA_2025/Simulink_ROS2/GeometricController.slx` and generated C++ | Sine-wave body-rate/thrust values sent to `/mavros/setpoint_raw/attitude` | Publishing scaffold; filename does not establish a functioning geometric feedback controller |

The AIAA 2025 generated `GeometricController.cpp` confirms sine-wave assignments to body rates/thrust and `type_mask=0`. The associated trajectory model publishes `/pose_ref`, while the controller model subscribes to `/pos_ref`; these files are not a ready-to-run matched controller pipeline.

Historical corroboration:

- `NGC/2022.04.25 Update/Tex Files/Presentation.tex`, around line 228, explicitly describes a preliminary simulation using 18 PID controllers.
- `NGC/20220926_Update/Tex Files/Presentation.tex`, around lines 395–416, describes a simplified peg-in-hole **simulation**, friction/alignment problems, and improved insertion with an operational-space controller.
- 2021 reports show custom P–PID/DOB flight-response plots, but no inspected manifest binds them to the public source snapshot above.

Thus, controllers have been found; the user's remembered well-performing physical peg-in-hole controller has not yet been uniquely identified. A video/date, MATLAB project entry point, or matching flight log would resolve the lineage more reliably than filenames.

## Verified Siemens boundary

Primary source: [EKF/WO plantout.m](../../backups/NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/plantout.m).

- Lines 1–8 reference a Windows Simcenter 3D 2206 Motion installation and the `am_motion_v001_ekf_wo-cosimulation_operational_space` solver case.
- Lines 47–51 construct a 53-output bus and an S-function named `vlmotionmex`, parameterized by `antype,motionfiles,feedthrough`.
- Line 163 selects eight inputs: `BODY_FX,BODY_FY,BODY_FZ,BODY_TX,BODY_TY,BODY_TZ,SERVO_T1,SERVO_T2`.
- The saved SLX's `simulink/systems/system_1612.xml` independently contains the same eight inputs and 53-output demultiplexer.
- Saved configurations include a 0.01 s fixed step. This does not establish the solver's allowable coupling rate, contact convergence, runtime speed, or PX4 lockstep compatibility.

The 53 outputs in this version are:

| Indices, one-based | Signals |
| --- | --- |
| 1–9 | X/Y/Z; VX/VY/VZ; AX/AY/AZ |
| 10–12 | YAW/PITCH/ROLL |
| 13–18 | Q1/Q2; VQ1/VQ2; AQ1/AQ2 |
| 19–24 | VRX/VRY/VRZ; ARX/ARY/ARZ |
| 25–30 | EE position and Euler orientation |
| 31–33 | VYAW/VPITCH/VROLL |
| 34–39 | EE linear and angular velocity |
| 40–43 | BODY_QUATW/X/Y/Z |
| 44–47 | EE_QUATW/X/Y/Z |
| 48–53 | EE linear and angular acceleration |

The older `20230605_ports.xlsx`, Sheet1 A1:A39/B1:B8, agrees on input names but not output count/order. For example, its output 13 is VYAW; the later generated script's output 13 is Q1. Reusing its indices could produce plausible-looking but wrong signals.

Signal names alone do not establish force units, reference point, orientation convention, angular-rate frame, or whether acceleration includes gravity. Those require solver/model inspection and controlled perturbation tests. In particular, a simulated IMU needs body-frame specific force at the sensor location, not an unexamined copy of AX/AY/AZ.

`SignalReqs.xlsx` lists 100 Hz companion/mocap/servo/RPM requirements and 1,000 Hz floating-base wrench computation. The requirements-met columns are empty. These are desired rates, not measured performance.

No FMI export or independently callable step API was verified. Siemens publicly describes Simulink and generic co-simulation capabilities, but installed-version support and licenses must be verified. [Siemens motion simulation guide](https://static.sw.cdn.siemens.com/siemens-disw-assets/public/KUCEhUJ9PnepHpqeVdINg/en-US/simcenter-3d-for-motion-simulation-sg-77919-d16.pdf)

## Actuation, arm, and other public work

[AIAA 2025 wrench_from_RPM.m](../../backups/NG_ASU_KT/AIAA_2025/RPM_FT_Data/wrench_from_RPM.m) maps six measured RPMs through per-motor squared-speed coefficients and a hexacopter geometry matrix to Fz/Tx/Ty/Tz. It is a useful propulsion-model starting point. It does not identify normalized PX4 command-to-RPM response, ESC lag, dead zones, or battery dependence. The physical motor order still needs matching.

The older Arduino AX-12A code exposes register operations via I2C address `0x10` and includes position/speed/load register handling. The separate `backups/dynamixel_setup/multi_servo_occilating_control.py` uses Dynamixel protocol 1.0, 1 Mbaud, IDs 7/16/19, goal-position writes and present-position reads. These are distinct setups; neither proves a deployed two-joint ROS arm interface. Simulator torque inputs will need a servo model if hardware commands are positions.

The public repository index contained 25 repositories. Six relevant repositories were downloaded for source review; eight additional research repositories were queried for file inventories, with selected sources read:

- `cleanrl_djif450` (`aa7bc8aeb554db2e4cec6f9df711e08d575c2001`) and `SB3_drone` (`54a04ba9a9e83b505bf46a4464f3ccdaf08dd0d7`) contain acceleration/residual-learning environments. The inspected CleanRL environment implements its own attitude/rate approximations and thrust/torque lag. It does not execute PX4 and is not controller-parity evidence.
- `quadrotor_modeling_simulating` contains a compact Python model; `Collaborative_Aerial_Transportation` contains a separate RotorS-based simulation/control stack. They may offer reusable methods but were not tied to this aircraft's deployment.
- `Pestana_Quad_IBVS_Controller_Replication_SITL` contains another SITL-oriented project; only its file inventory was reviewed.
- `EGR_608_Semester_Project` and `DSCC_2020` were inventoried, not treated as current flight software.
- The API returned HTTP 409 for the `data-driven-system-identification` tree; no source content was recovered from it.

The remaining repositories were screened by name/metadata; they were not all exhaustively audited. No external source was executed.

## Remaining inspection needed before selecting the boundary

1. On the aircraft: firmware build/version, full parameter export, motor/output mapping, actual airframe/arm geometry, active services and launch files, workspace overlays, and controller source revision.
2. With the system disarmed: ROS node/topic/type/QoS inventory; bounded samples of mocap, estimated state, status, commands, actuator/RPM and arm streams; actual rates and timestamp relationships.
3. In Siemens/MATLAB: open the selected model, regenerate its interface, confirm the 53-port map and units/frames, run isolated force/torque tests, inspect contact outputs and available sensor locations, and measure coupling/runtime behavior.
4. Recover provenance for the successful controller: matching video/date, logs, gains, source/SLX revision, hardware configuration, and experiment trajectory.

See [the revised architecture proposal](CONTROL_ARCHITECTURE_PROPOSAL.md) for how these findings change the design.
