# Interface and convention contract — v1 workbench

Status: agreed architecture encoded as message schemas; runtime enforcement is
future work. See `WORKBENCH.md` for implemented capabilities. The canonical field
lists are `src/uam_interfaces/msg/*.msg`; do not maintain duplicate message schemas.

## Boundaries and ownership

Trajectory -> research controller -> limiter -> PX4 adapter -> PX4 -> plant.
State flows through a PX4-specific adapter into the normalized state interface.
The mathematical core depends on none of ROS, PX4, Gazebo, or Siemens.

The physical UAM is a primary target. The existing Gazebo X500 is a temporary
integration and safety test fixture, not a UAM dynamics model or a significant
model-development target. Use it to validate frames, timing, PX4 semantics,
lifecycle, failure handling, and initial closed-loop control before hardware use.
After numerical acceptance, pursue physical-system auditing and baseline data
collection alongside SITL validation where practical. When Siemens access is
available, start restoration and model understanding without waiting for Gazebo
polish. The intended simulator path is PX4 SITL + Simcenter, subject to feasibility.
No Siemens bridge is implemented in this milestone.

## Frames and physical quantities

- `odom`: fixed local world, ENU convention, metres. Geographic alignment is not
  implied for arbitrary-heading mocap. `base_link`: body FLU.
- Position and translational velocity/acceleration use `odom`.
- Angular velocity uses `base_link`, radians/second.
- ROS quaternion order is x,y,z,w. Define R by v_world = R * v_body. Validate unit
  norm; do not treat a zero quaternion as a valid orientation.
- Desired acceleration is inertial dv_world/dt, m/s^2. It is NOT IMU specific
  force and has NO extra gravity term. PX4 handles gravity/hover compensation.
- Yaw is radians about world +Z; yaw rate is its derivative. Both are finite in
  v1. Keep their trajectory evolution consistent; yaw-rate-only control is not v1.
- NED to ENU vector conversion: (x,y,z) -> (y,x,-z). FRD to FLU: (x,y,z) ->
  (x,-y,-z). R_ENU_FLU = T_ENU_NED * R_NED_FRD * T_FRD_FLU.
- yaw_ENU = wrap(pi/2 - yaw_NED); yaw_rate_ENU = -yaw_rate_NED.
- Unknown or unsupported odometry frame declarations are rejected, not relabeled.
  Arbitrary-heading FRD world frames need an explicitly configured transform.
- A standard nav_msgs/Odometry visualization output would require twist expressed
  according to that message's child-frame convention; our custom world velocity
  must not be copied blindly into it.

## Messages and topics

| Topic | Type | Meaning |
| --- | --- | --- |
| /uam/reference | TrajectoryReference | Analytic p/v/a and yaw at evaluation time |
| /uam/state | EstimatedState | Normalized estimate plus validity and reset metadata |
| /uam/command/requested | AccelerationYawCommand | Raw controller request |
| /uam/command/limited | AccelerationYawCommand | Limited command eligible for adapter use |
| /uam/controller/diagnostics | ControllerDiagnostics | Errors, contributions, limits, rejection |

Custom messages are justified by expiry, provenance, acceleration, reset and
contribution semantics absent from individual standard messages. Standard geometry,
header, and time fields are reused. PX4 message types must remain adapter-local.
The core's plain C++ types carry only mathematics; wrappers add metadata.

All values must be finite when their validity flag is true. Invalid state fields
may be NaN; consumers must check validity first. References and commands have no
partial-control semantics: all their numeric fields must be finite. NaNs used to
disable PX4 fields are created only inside the PX4 adapter.

Sequence IDs increase within each producer session/run. A future run manifest
identifies the session. Requested and limited commands share a command ID and
retain state/reference IDs; limiting must not refresh generation time or expiry.
Diagnostics for rejected computation have command_valid=false; their command ID
must not be interpreted as evidence of a published command.

EstimatedState uses separate translation, attitude, angular-velocity and validity
sample times so asynchronous data is not presented as synchronized. Source reset
counters are retained; the adapter increments frame_epoch upon any relevant reset,
restart, or origin change. A frame epoch mismatch aborts the current run and
requires explicit reinitialization/rebasing. No automatic origin chasing.
Uncertainty and additional raw metadata remain in recorded source messages for v1.

## Time and freshness

Initial real-time SITL uses system ROS time, use_sim_time=false. Future simulated
clock use must be explicit and consistent across the run, never mixed silently.
The runtime wrapper owns T0 and supplies elapsed seconds to the pure trajectory
library. A clock jump aborts/reinitializes the experiment. Monotonic local receive
time drives watchdogs; it is never compared numerically with ROS timestamps.

Reference header.stamp is evaluation time, command header.stamp is generation time,
and state header.stamp is translation sample time normalized to the chosen ROS
clock. received_at is ROS reception time, not a replacement for source time.
PX4 DDS performs timestamp offset conversion; verify the observed domain before
normalizing it again. Record raw as-received timestamps and synchronization status.

Future runtime checks require source age, monotonic receive age, stream skew,
valid_until, matching frame epochs, and estimator validity. Repeated source stamps
must not masquerade as new estimates. Clock synchronization uncertainty inhibits
control. Expired commands must not be republished with a fresh expiry. Timeout
numbers remain unset until measured and tested; no physical limits are assumed.

## PX4 contract for the pinned development baseline

Pinned PX4: d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
Pinned px4_msgs: 86d8239e962f6939e05c3737784f60c02fa884db.

Adapter intent: OffboardControlMode acceleration=true, every other flag=false;
TrajectorySetpoint position/velocity/jerk NaN, finite NED acceleration, yaw and
yawspeed. PX4 incorporates gravity and hover thrust. Yawspeed is world-Z rate
feedforward alongside yaw tracking. Proposed runtime rate is 50 Hz; offboard
proof-of-life requires >2 Hz and an established stream before mode entry.

Stopping commands with a live offboard heartbeat is not a reliable failsafe.
Adapter expiry must revoke healthy command forwarding and perform a tested
mode/authority transition; observe acknowledgments and actual vehicle status.
COM_OF_LOSS_T and COM_OBL_RC_ACT influence loss behavior. Do not assume landing,
and do not use automatic disarm as a generic in-flight fallback.

The pinned offboard health check tests attitude validity for the acceleration flag,
while v1.17 documentation describes a velocity-estimate requirement. Independently
require valid position, velocity and attitude for our tracking controller; test
full commander behavior in the actual configured system. VehicleOdometry quality
is unused in this baseline and is not a standalone validity test.

Source evidence:
- https://docs.px4.io/v1.17/en/flight_modes/offboard
- https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
- https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
- https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp
- https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em

## Limits, lifecycle and reproducibility (future runtime)

Before command forwarding require initialization, valid state, configured limits,
a valid unexpired reference, and explicit experiment activation. Abort on stale
state/commands, resets, invalid estimates, mode loss, or experiment abort. Bound
trajectory position/speed, limit acceleration and yaw rate, monitor measured
speed and position. Acceleration clipping alone cannot enforce a speed bound.
Safe authority transfer is backend/operator-specific and must be tested.

Record reference -> errors -> requested command -> limited command -> PX4 status
and exposed setpoints -> actuators where available -> estimates, raw mocap and
simulation truth where available. Publication is not PX4 acceptance: trajectory
setpoints have no individual acknowledgment. Default DDS does not expose every
actuator/internal setpoint; plan ULog plus deliberate telemetry configuration.

Manifests should include run ID, time, resolved experiment, commit/dirty state,
gains, limits, backend, PX4 version/parameters, arm configuration and bag/log paths.
Tracking error and model prediction error are separate analysis quantities.
Future backend names belong to experiment/adapter configuration, never PD math.

## Historical provenance

This milestone does not copy JJ's controller code or treat historical P-PID gains
as PD defaults. Prior findings are in CONTROL_INTERFACE_AUDIT.md and
JJ_V004_PARAMETER_AUDIT.md. Exact successful-flight commit/config/log provenance
remains unresolved. Document source, retained behavior, modifications and reasons
before subsequently reusing historical code.
