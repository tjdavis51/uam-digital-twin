# Controller parity and the Siemens integration decision

Status: proposal for review, 2026-09-29. Based on the [static interface audit](CONTROL_INTERFACE_AUDIT.md). The command level and Siemens runtime integration remain unselected.

## Recommendation

Treat PX4 SITL against Simcenter as the leading candidate for end-to-end real-versus-sim experiments, subject to a bounded feasibility prototype. Keep the project interface independent of the backend, and support a separately labeled approximate simulation backend when useful. Do not claim that an adapter makes two different inner controllers equivalent.

Two independent decisions are required:

1. Where does the research controller enter PX4: position, velocity, acceleration, attitude/thrust, or lower?
2. How much of PX4 is executed on the simulation branch?

Choosing acceleration commands does not eliminate the second decision. PX4 still performs command conversion, stabilization, allocation, and related filtering/limiting. Replacing those with ideal acceleration tracking changes the system being measured.

## Comparison of alternatives

| Approach | What can be held common | Main mismatch or cost | Appropriate use |
| --- | --- | --- | --- |
| PX4 SITL + Siemens | Research controller plus retained PX4 algorithms, configuration, estimator and allocation, when deliberately matched | Sensor/actuator models, timing, scheduling/build differences, and custom bridge work remain | Preferred candidate for validating the complete deployed stack |
| Same acceleration or attitude/thrust command; independently reproduced inner loops | Research controller and explicit command semantics | Reimplementation drift in filters, integrators, limits, hover-thrust behavior and timing | Practical fallback if independently characterized and error is acceptable |
| Extract actual PX4 control modules into a simulator harness | Selected source algorithms | Module dependencies, scheduling, parameter/state initialization, estimator and allocator omissions | Possible intermediate approach; not automatically simpler than SITL |
| Identified closed-loop model below an acceleration/attitude/rate boundary | Measured command-to-state behavior over a stated operating envelope | Result represents aircraft plus retained controller; fidelity can degrade with arm posture/contact/battery changes | Fast learning or outer-loop experiments where that aggregate model is the target |
| Shared external stabilization, PX4 primarily for actuation/management | More research-control code can be identical | Large flight-control responsibility; normalized actuation is still not physical torque | Only if low-level stabilization becomes a research requirement |
| Recorded-input replay into Siemens | Physical model receives reconstructed realized input without a competing simulation controller | Input and initial-state uncertainty; open-loop divergence; does not test closed-loop stability | Complementary plant-identification experiment even if SITL is unavailable |

A body-rate boundary reduces the number of hidden loops but does not remove the rate controller or allocation. An attitude boundary still retains attitude/rate feedback. No choice of name or message type makes those dynamics disappear.

## Candidate architecture with PX4 on both branches

```text
trajectory + shared research controller + command limiter
                         |
                  shared PX4 adapter
                  /                \
        PX4 on flight controller   matched PX4 SITL
        retained loops + EKF       retained loops + EKF
                  |                |
        real ESCs/motors/servos    actuator/servo model
                  |                |
        physical aircraft/arm     body wrench + joint torques
                  |                |
                  |               Siemens through verified co-simulation
                  |                |
        real IMU + mocap/etc.      synthetic IMU + mocap/etc.
                  |                |
                  +--> respective PX4 estimators
                              |
                    shared estimated-state adapter
                              |
                     controller + recorder
```

The experiment interface sits above PX4; the simulator physics interface sits below it. These should be separate interfaces. Simcenter is not expected to consume the same ROS acceleration setpoint that PX4 consumes.

PX4 documents a simulator API exchanging simulated sensors and actuator outputs, including a MAVLink-based path and lockstep operation. That provides a real integration mechanism to investigate; the existence of this API does not prove the Siemens coupling works. [PX4 simulation documentation](https://docs.px4.io/v1.17/en/simulation/)

The currently evidenced Siemens path is Simulink's `vlmotionmex`, not a verified FMU or Python step function. A first prototype can investigate a Simulink-hosted bridge to PX4 rather than replacing Siemens integration tooling immediately. A generic co-simulation API or FMI route remains an alternative only after version/license/runtime verification.

## What the bridge must model

### From PX4 to Siemens

1. Receive actuator outputs in their actual units/order and armed/disarmed semantics.
2. Convert normalized control/PWM-equivalent values into rotor speeds or forces using an identified actuator model. Do not equate normalized output with newtons or RPM.
3. Apply motor lag, limits and relevant battery effects.
4. Sum rotor forces and moments about the correct body reference point and populate Siemens body-wrench inputs. A fixed-rotor vehicle must not gain independent lateral body-force authority merely because the Siemens input bus has Fx/Fy fields.
5. Convert arm position/velocity commands into servo torques using a separate servo model when required.

The archived RPM-to-wrench fit is a starting point for step 4, not a solution to steps 1–3. Representing propulsion by a resultant body wrench is a useful initial approximation, but does not by itself capture rotor-local aerodynamic or flexible-body effects.

### From Siemens to PX4

Use truth to synthesize the measurements needed by the matched estimator configuration. For an indoor mocap setup, model IMU plus delayed/noisy external pose, and any other sensors actually fused by that configuration. Do not invent GPS dependence or disable required estimator paths merely to make the simulation run.

For the IMU, determine acceleration at the sensor location, account for gravity and body orientation, and distinguish angular velocity from Euler-angle rates. For mocap, preserve the laboratory frame, marker offset, source timing, delay and dropout semantics. Truth remains separately available for diagnosis; it is not silently substituted for estimated feedback.

### Time and solver ownership

Specify one simulation-time authority, exchange order, hold behavior between actuator updates, sensor sample times, and reset behavior. Verify PX4 and ROS trajectory clocks advance consistently when Siemens pauses or runs slower than wall time. Define which component waits for which message to avoid a lockstep deadlock.

The archived 10 ms Simulink configuration is not sufficient evidence for the required inner-loop/sensor exchange rates. Test substepping or a smaller exchange interval and demonstrate convergence as the interval is reduced, especially with contact. If lockstep cannot be supported, a real-time arrangement needs measured throughput/jitter and a stated timing-error budget.

## What “same PX4” must mean

Record and reconcile firmware source revision, relevant modules/build options, controller/estimator parameters, airframe and actuator geometry, output ordering, command mode, filter configuration, controller update behavior, hover-thrust logic and initial conditions. Hardware calibration parameters cannot necessarily be copied blindly into synthetic sensors; document every justified difference.

SITL removes a major source of controller mismatch. It does not make remaining trajectory error exclusively physics error. Sensor models, estimation, communications, initial state, numerics and hardware scheduling still contribute. Hardware-in-the-loop could later study FCU execution effects, but is not a prerequisite for this first decision.

Use matched gains/limits for the primary comparison. Retuning simulation independently to fit the real trajectory can conceal a deficient plant model. A separately labeled retuned experiment may be useful, but answers a different question.

## Research interfaces to own

Keep these logical contracts separate even if several initially share a ROS process:

| Contract | Proposed contents |
| --- | --- |
| Reference | Position/velocity/acceleration, yaw/yaw rate, arm reference, experiment time and trajectory version |
| Research command | Explicit mode, units/frame, source time, expiry, requested and limited values; start with only modes we validate |
| Estimated state | Pose, linear/angular velocity, validity, uncertainty, frame/origin, sample time, estimator reset information |
| Authority/status | Mode/armed status, command acknowledgments, health, available capabilities and stale-data status |
| Arm command/state | Named joints, explicit position/velocity/effort semantics, units, limits and sample times |
| Simulator physics | Ordered actuator inputs, physical wrench/joint torque contract, truth state and sensor-generation metadata |
| Experiment record | Run/configuration IDs, code and parameter versions, clock mapping, event markers, requested/limited/PX4/actuator signals, mocap/estimator/arm/contact data and quality report |

The PX4 adapter should own command translation and offboard lifecycle. The controller should not publish PX4 mode heartbeats independently. A heartbeat must not mask a dead controller: expired commands need an explicit tested authority/fallback policy.

At this stage, acceleration plus yaw is a strong candidate because it matches a recovered controller lineage, not a selected requirement. Position control remains the initial infrastructure baseline. Attitude/thrust is an alternative if the research needs explicit thrust mapping or coupling compensation beyond what the acceleration boundary permits.

## Experimental strategy for attributing mismatch

Use two complementary experiment families:

1. **Matched closed-loop runs:** same research controller, reference, matched PX4 configuration, arm program and comparable initial state. Compare tracking, estimates, internal setpoints, saturation, actuators and timing. Diagnose residual differences rather than calling every difference a plant error.
2. **Short-horizon recorded-input replay:** initialize the simulator from measured state and replay measured RPM/actuator information and arm commands where available. Compare predicted state evolution with observations. This avoids a different simulation controller generating a different input, though actuator reconstruction and measurement error remain.

Log multiple input levels. A desired acceleration is a controller request; motor RPM or an identified rotor wrench is closer to the physical input needed for parameter identification. Synthetic sensors should be disabled or separately controlled in a physics-only replay analysis, rather than mixing sensor and plant error without explanation.

## Revised sequence and decision gates

1. **Complete live deployment inventory.** Obtain the missing aircraft evidence listed in the audit. Acceptance: one documented command path with actual firmware/schema compatibility, state source, authority owner and current airframe configuration.
2. **Recover a controller baseline.** Tie the P–PID or MATLAB candidate to logs/gains/demo provenance; compare equations and timing. Preserve historical source and implement no blind copy. Acceptance: replayable reference/state-to-command behavior with known limitations.
3. **Implement state/logging/experiment infrastructure after design agreement.** Use existing Gazebo/PX4 SITL to validate frame, clock and recording contracts before Siemens is available. Acceptance: repeatable gentle baseline run with complete required signals and stale-data handling.
4. **Restore one Siemens case.** Confirm units, frame, reference points, port order, actuator authority and contact behavior with isolated inputs. Acceptance: trustworthy force/torque-to-state results through the installed solver interface.
5. **Prototype PX4–Siemens coupling.** First use a minimal rigid-body case and fixed arm; synthesize stationary sensors, check estimator behavior, verify actuator signs/order, then test simulated stabilization. Measure exchange timing, resets, repeatability and step-size sensitivity before full UAM/contact integration.
6. **Choose the production simulation path.** Prefer SITL if the bridge is reproducible and timing/numerics are acceptable. If it is disproportionate or unsupported, explicitly choose either an identified closed-loop backend or verified module reuse, with a limited scientific claim and independent error characterization.
7. **Select the research control boundary.** Use hardware capability and the identification/contact objective to choose acceleration, attitude/thrust or another validated mode. Record which loops remain in PX4 and which code is common across backends.
8. **Run matched and replay experiments, then add moving-arm/contact tasks.** Add learned residuals only after the baseline and mismatch attribution work; log baseline, residual and limited total separately.

The immediate deliverable is a verified interface/ownership specification and controller provenance, followed by a small Siemens–PX4 feasibility experiment. Full simulator coupling and a new flight controller should not be prerequisites for starting the data infrastructure.
