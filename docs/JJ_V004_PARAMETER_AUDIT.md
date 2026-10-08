# JJ V004 / Current UAM Parameter Audit

**Audit date:** 2026-09-21  
**Scope:** `/Users/tjdavis/Lab/NG_ASU_KT` (MECC 2023 V003/V004 branches, hardware software, and AIAA 2025 RPM/F/T archive)  
**Target:** first primitive MuJoCo model: free base + two revolute arm joints + six rotor wrench sources

## Executive verdict

There is enough information to build a useful **MuJoCo v0.1 validation model**, but not enough to claim that it is already a metrically accurate model of the lab's present physical aircraft.

The archive contains three distinct parameter sets that must not be silently merged:

1. **V004 EKF/WO reduced model (2023):** the strongest source for the standard two-link arm and the original V004 airframe. Its precise masses and diagonal inertias also occur in the Simcenter result model, suggesting CAD/Simcenter-derived rather than hand-tuned values.
2. **V004 ACC configuration (2023 model, used through 2024/2025 work):** a longer, heavier end link and different airframe mass/inertia, apparently representing the collision-avoidance end-effector-stick configuration.
3. **AIAA 2025 F/T test article:** the strongest source for current six-rotor geometry and measured RPM-to-wrench behavior, but the paper describes a DJI F550 with Pixhawk 6X, prop guards, Raspberry Pi 4B, 960 KV motors, 35 A ESCs, and 9.45-inch propellers. That is not demonstrably identical to the V004 CAD assembly, which contains a Pixracer and older component geometry.

Therefore the recommended seed is:

- use the **V004 EKF/WO** arm geometry and link inertial data;
- use the **AIAA 2025** rotor radius, motor ordering implied by the control-allocation matrix, and per-motor thrust coefficients;
- treat the base mass/inertia as provisional until the exact current hardware configuration is defined and weighed/CAD-recomputed;
- expose rotor speed (RPM) directly in v0.1, rather than inventing a normalized-command-to-RPM model;
- use ideal torque or position servos for v0.1, because the archive does not establish a validated two-joint V004 servo model.

The smallest questions that prevent a *confident physical twin* are:

1. Which physical build is the target: standard 66 mm second link, ACC 196 mm stick, or another current assembly?
2. What are the current assembled base mass, CoM, and inertia after the Pixhawk 6X/prop-guard/hardware changes?
3. How do physical motor labels 1–6 map to the six arms and to the PX4 actuator-output order, and which arms are CW versus CCW?
4. What interface does the lab PX4–MuJoCo bridge expect (normalized actuator, angular speed, thrust, and which frame/order)?

None of these blocks construction of the skeleton; they block calling it a validated replica of the current aircraft.

## Evidence and authority rules

Confidence labels in this document mean:

- **High:** directly encoded and independently corroborated within the same configuration.
- **Medium:** directly encoded, but provenance/configuration mapping is incomplete.
- **Low:** inferred from geometry, comments, filenames, or a different test article.

Source preference for the initial model is:

1. configuration-specific generated/Simcenter data;
2. configuration-specific builder source;
3. experimentally identified AIAA 2025 data;
4. V003 analytical code as a regression reference;
5. comments, exploratory scripts, and generic component code.

File dates and folder names identify development branches, not guaranteed hardware release tags. No manifest in the archive declares one branch to be the final physical configuration.

## Configuration timeline and conflicts

| Configuration | Geometry | Base | Link 1 | Link 2 | Interpretation |
|---|---:|---:|---:|---:|---|
| V003 analytical (2023-04) | 0.35905 + 0.045341 + 0.06674 m | 1.6 kg; 0.1 kg m² diagonal | 0.05 kg | 0.06 kg | Early analytical model; several inertias are rough assumptions. |
| V004 FA/UA ML (2023-06/07) | 0.36 + 0.044 + 0.066 m | 1.6 kg; 0.1 kg m² diagonal | 0.05 kg | 0.06 kg | Rounded reduced model. |
| **V004 EKF/WO (2023-10 builder; Simcenter-correlated)** | **0.36 + 0.044 + 0.066 m** | **1.72557981175 kg** | **0.05416589281 kg** | **0.06346757869 kg** | Best standard-arm V004 seed. |
| V004 ACC (2023-09 RBT; later ACC work) | 0.36 + 0.044 + **0.196 m** | 1.6666 kg | 0.05 kg | **0.1474 kg** | Extended/heavier collision-avoidance end-effector configuration; not interchangeable with standard V004. |
| AIAA 2025 F/T article | rotor radius 0.275 m | full mass/inertia not reported | manipulator absent in shown test article | manipulator absent | Best propulsion identification, not a complete V004 inertial model. |

## Coordinate and state conventions

### Generalized coordinates

The reduced MATLAB tree uses

```text
q = [x_b, y_b, z_b, yaw, pitch, roll, q1, q2]^T
```

The serial virtual joints are `x`, `y`, `z`, `z`, `y`, `x`, `x`, `y`, followed by a fixed end-effector joint. This creates the base rotation

```text
R_I_B = Rz(yaw) Ry(pitch) Rx(roll)
```

and arm axes `q1` about local +X and `q2` about local +Y. Positive rotation is right-handed. Evidence: [V004 EKF/WO builder](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L5-L17) and [V003 FK](../../NG_ASU_KT/MECC_2023/AerialManipulatorV003/FK_func.m#L1-L21). **Confidence: high for JJ's reduced model.**

The aircraft/body and AIAA wrench convention is NED/FRD-like: +X forward, +Y right, +Z down. Gravity is `[0, 0, +9.80665]` m/s². The AIAA processing explicitly applies `diag(1,-1,-1)` to obtain NED-like coordinates. MuJoCo normally uses +Z up, so the implementation must contain an explicit world/body-frame adapter instead of changing signs ad hoc.

### Inertia ordering

MATLAB `rigidBody.Inertia` uses:

```text
[Ixx, Iyy, Izz, Iyz, Ixz, Ixy]
```

All retained products of inertia are zero. Values below are about each link CoM and expressed in its link frame as encoded by the builder. Whether zero products reflect true principal axes or deliberate diagonalization is not documented.

## Standard V004 rigid-body parameter register

The following values come from the V004 EKF/WO builder and are embedded in its generated `rbtForCodegen.m`; the same exact masses/inertias are visible in the corresponding Simcenter `.mres`. This makes them the best standard-arm reduced model, not necessarily the current 2025 airframe.

| Parameter | Numerical value | Unit | Frame/convention | Provenance and source | Configuration / conflicts | Confidence for current UAM |
|---|---:|---|---|---|---|---|
| Base mass | 1.72557981175 | kg | body L6 | CAD/Simcenter-derived appearance; [builder L35](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L35) | V003/early V004: 1.6; ACC: 1.6666 | Medium-low until present aircraft is weighed |
| Base CoM | `[0,0,0]` | m | body frame | Encoded assumption; [builder L45-L47](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L45-L47) | Every reduced branch places base CoM at origin | Low as a physical measurement; high as JJ-model convention |
| Base inertia | diag(`[0.0155369497728404, 0.0546619200436569, 0.0689651793064483]`) | kg m² | about base CoM, body axes | Precise CAD/Simcenter-derived value; [builder L41](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L41) | V003/early V004: diag(0.1); ACC: diag(0.027,0.052,0.076) | Medium-low for present build; high for standard EKF model |
| Link 1 mass | 0.05416589281 | kg | link L7 | CAD/Simcenter-derived appearance; [builder L35](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L35) | Rounded branches/ACC: 0.05 | Medium-high if same arm |
| Link 1 CoM | `[0.03,0,0]` | m | from joint-1/link origin | [builder L45-L47](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L45-L47) | Same across branches | Medium-high |
| Link 1 inertia | diag(`[8.37781161428309e-6, 5.99180768376784e-5, 6.20839810820291e-5]`) | kg m² | about link CoM, link axes | CAD/Simcenter-derived appearance; [builder L42](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L42) | Rounded V004/ACC: `[8,62,60]e-6`; V003 analytical: `[8,17,15]e-6` | Medium-high |
| Link 2 mass | 0.06346757869 | kg | link L8 | CAD/Simcenter-derived appearance; [builder L35](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L35) | Rounded: 0.06; ACC extended: 0.1474 | High only for standard short link |
| Link 2 CoM | `[0.066,0,0]` | m | from joint-2/link origin | [builder L45-L47](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L45-L47) | ACC: `[0.083,0,0]`; odd because short-link CoM equals EE offset—verify CAD body origin | Medium |
| Link 2 inertia | diag(`[2.71594246243794e-5, 8.03226909188509e-5, 6.70316298635848e-5]`) | kg m² | about link CoM, link axes | [builder L43](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L43) | Rounded V004: `[17,291.36,291.36]e-6`; V003 analytical: `[17,30,30]e-6`; ACC: `[37,1361,1348]e-6` | Medium-high for short link |
| Products of inertia | all zero | kg m² | link frames | Builder values | Could be principal-axis reduction rather than full CAD tensor | Medium |

### ACC alternative (do not blend with standard link 2)

The ACC builder changes base mass/inertia, link-2 length/mass/CoM/inertia together. It should be selected as one coherent configuration:

- base: 1.6666 kg, CoM `[0,0,0]` m, inertia diag(`[0.027,0.052,0.076]`) kg m²;
- link 1: 0.05 kg, CoM `[0.03,0,0]` m, inertia diag(`[8,62,60]e-6`) kg m²;
- link 2: 0.1474 kg, CoM `[0.083,0,0]` m, inertia diag(`[37,1361,1348]e-6`) kg m²;
- joint-2 to end-effector: 0.196 m.

Source: [ACC builder L11-L45](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_ACC_2024/UAM_RBT_Dev/UAM_RBT_Builder.m#L11-L45). The folder also contains `EE_Stick.prt`, supporting (but not proving) the extended-tool interpretation. **Provenance: likely CAD-derived/analytically reduced; confidence medium for ACC, low for the standard present aircraft.**

## Kinematic parameter register

All transforms below have identity rotation at zero and translation along the parent +X axis.

| Parameter | Value | Unit | Convention | Source / provenance | Conflict | Confidence |
|---|---:|---|---|---|---|---|
| Aircraft → joint 1 | `[0.36,0,0]` | m | parent body to J7 fixed transform | [V004 builder L8-L10](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L8-L10), analytically encoded | V003 uses 0.35905 m | High for V004 reduced model |
| Joint 1 → joint 2 | `[0.044,0,0]` | m | J7/L7 to J8 fixed transform | same source | V003 uses 0.045341 m | High for V004 reduced model |
| Joint 2 → EE, standard | `[0.066,0,0]` | m | fixed L8→L9 transform after q2 | same source | V003 0.06674; ACC 0.196 | High for short-link V004; configuration-dependent |
| Joint 2 → EE, ACC | `[0.196,0,0]` | m | `0.6 - 0.404` | [ACC builder L11-L13](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_ACC_2024/UAM_RBT_Dev/UAM_RBT_Builder.m#L11-L13) | Standard 0.066 | High for ACC branch |
| Joint 1 axis | `[1,0,0]` | — | local +X, right-hand positive | `jointAxes(7)='x'`; [builder L5-L7, L70-L80](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L5-L80) | Simcenter lists local joint axes in its own marker frames; do not compare raw vectors without marker transforms | High |
| Joint 2 axis | `[0,1,0]` | — | local +Y, right-hand positive | `jointAxes(8)='y'`; same source | none in reduced model | High |
| Joint zero | identity rotation | rad | links collinear along body +X | builder fixed transforms and [V003 FK](../../NG_ASU_KT/MECC_2023/AerialManipulatorV003/FK_func.m#L12-L21) | physical encoder offsets absent | High for mathematical model; missing for hardware encoders |
| Joint limits | `[-150°, +150°]` each | deg | q1, q2 | [builder L14-L17](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L14-L17) | likely software/model assumption; AX-12A raw range code uses 0–1023 | Medium |
| Joint sign/offset to servo command | missing | — | must map servo raw/physical axes to q1/q2 | no authoritative V004 mapping found | exploratory AX-12A code uses raw home 511/512 | Low / blocking for hardware correlation, not v0.1 dynamics |

The RBT locks base pitch and roll to approximately zero (`±1e-10`) in the builder used for some planar/controller studies. This is a modeling constraint, **not** a physical aircraft limit and should not be copied into a free-base MuJoCo model.

## `UAM_RBT_Builder.m` and `rbtForCodegen.m`

### Origin and generation

`UAM_RBT_Builder.m` constructs a nine-body serial `rigidBodyTree`, saves `UAM_RBT.mat`, and calls:

```matlab
writeAsFunction(UAM, "rbtForCodegen")
```

See [EKF builder L49-L106](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m#L49-L106). `CreateRBTForCodeGen.m` is an alternate regeneration path that loads `UAM_RBT.mat` and invokes the same generator.

The generated EKF/WO file identifies its generation time as 2023-09-28 18:05:40 and reconstructs the exact body tree at runtime. It is consumed by the forward-dynamics functions, including [ODE_45_forward_dynamics.m](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/ODE_45_forward_dynamics.m), which calls `massMatrix`, `velocityProduct`, and `gravityTorque`, then solves

```text
qdd = M(q)^-1 [tau + wrench - C(q,qdot) - G(q)]
```

### Authority assessment

`rbtForCodegen.m` is the best machine-readable reduced rigid-body definition **within its branch**. It is not globally authoritative because files with the same name exist in multiple V004 branches and embed different parameter sets. The builder is preferable for review because it exposes units, comments, and generation logic. The generated file is preferable for verifying what MATLAB code generation actually instantiated.

The precise EKF/WO values also occur in `am_motion_v001_ekf_wo-cosimulation_operational_space.mres`, providing independent branch-level corroboration. The ACC generated tree instead contains the extended/heavy link values. The archive therefore supports selecting a named configuration, not selecting “the newest `rbtForCodegen.m`” by filename.

Two caveats affect validation:

- `SymbolicForwardKinematics.m` replaces revolute fixed transforms with pure rotations and is not a safe full-FK oracle.
- `UAM_RBT_Kinematics_Checker.m` appears to query the wrong body/sample in several lines. Prefer MATLAB Robotics System Toolbox calls directly on `UAM_RBT.mat` or `rbtForCodegen()`.

## Six-rotor geometry and propulsion

### AIAA 2025 nominal RPM-to-wrench model

The most mature standalone implementation is [wrench_from_RPM.m](../../NG_ASU_KT/AIAA_2025/RPM_FT_Data/wrench_from_RPM.m#L1-L24):

```text
f_i = k_T,i RPM_i^2
[Fz, Tx, Ty, Tz]^T = CAM [f_1 ... f_6]^T
```

The units are forced by the code and paper:

- `RPM_i`: revolutions per minute (not rad/s);
- `k_T,i`: N/RPM²;
- `f_i`: N, positive scalar thrust magnitude;
- `l`: m;
- `k_tau = Q/T`: m, despite being called a “torque coefficient”;
- output: N, N m, N m, N m.

This `k_tau` must not be confused with an alternate direct coefficient in `ComputeTorqueConstant.mlx` that multiplies RPM² and therefore has units N m/RPM².

### Rotor layout

With body +X forward, +Y right, +Z down, all thrust axes are body `-Z`. The CAM implies these hub positions:

| Motor | Position `[x,y,z]` (m), `l=0.275` | Thrust axis | Yaw reaction sign in CAM | Physical CW/CCW |
|---:|---|---|---:|---|
| 1 | `[+0.238157, +0.137500, 0]` | `[0,0,-1]` | `+` | Not independently verified |
| 2 | `[0, +0.275000, 0]` | `[0,0,-1]` | `-` | Not independently verified |
| 3 | `[-0.238157, +0.137500, 0]` | `[0,0,-1]` | `+` | Not independently verified |
| 4 | `[-0.238157, -0.137500, 0]` | `[0,0,-1]` | `-` | Not independently verified |
| 5 | `[0, -0.275000, 0]` | `[0,0,-1]` | `+` | Not independently verified |
| 6 | `[+0.238157, -0.137500, 0]` | `[0,0,-1]` | `-` | Not independently verified |

These positions are an algebraic inference from `r × [0,0,-f_i]` and the CAM, not a direct CAD extraction. The motor sequence is the sequence of columns in JJ's CAM and the sequence of `/mavros/esc_telemetry/telemetry` entries copied into `RPM(:,1:6)` by `TestDataCompilation.m`. A physical arm-label diagram was not found. Preserve the yaw **signs** initially; do not label them CW/CCW until visually checked against props or the PX4 geometry.

The exact allocation matrix is:

```text
CAM = [
 -1,          -1,          -1,          -1,          -1,          -1;
 -l sin(30°), -l,          -l sin(30°), +l sin(30°), +l,          +l sin(30°);
 +l sin(60°),  0,          -l sin(60°), -l sin(60°),  0,          +l sin(60°);
 +k_tau,      -k_tau,      +k_tau,      -k_tau,      +k_tau,      -k_tau
]
```

This maps six **positive thrust magnitudes** to `[Fz,Tx,Ty,Tz]`; total `Fz` is negative because lift acts along `-Z`.

### Per-rotor thrust coefficients

| Motor | Recommended nominal `k_T` (N/RPM²) | Other archived values | Status/confidence |
|---:|---:|---|---|
| 1 | 1.125e-7 | `kT.mat`: 9.695211019e-8; optimizer seed: 1.095104105e-7 | Conflicting; medium |
| 2 | 0.950e-7 | `kT.mat`: 1.043077604e-7; seed: 1.235641468e-7; visualization: 0.975e-7 | Conflicting; medium |
| 3 | 0.950e-7 | `kT.mat`: 1.022175076e-7; seed: 0.945795252e-7 | Conflicting; medium |
| 4 | 0.950e-7 | `kT.mat`: 1.008389090e-7; seed: 0.850935356e-7 | Conflicting; medium |
| 5 | 0.900e-7 | `kT.mat`: 0.968946664e-7; seed: 1.237955439e-7; visualization: 0.925e-7 | Conflicting; medium |
| 6 | 1.125e-7 | `kT.mat`: 0.959974551e-7; seed: 1.063113009e-7 | Conflicting; medium |

The “recommended nominal” column is recommended only because it is the final standalone function used by downstream AIAA processing. The archive does not save the final `fmincon` result that proves those rounded numbers came from the shown run.

### How coefficients were obtained

[compute_constants.m](../../NG_ASU_KT/AIAA_2025/RPM_FT_Data/compute_constants.m#L1-L155) loads processed run `0072_20241020_1555`, documented in the data-collection workbook as a thrust/torque constant calibration run. It filters at 1 Hz with a sixth-order Butterworth filter at 100 Hz, then uses constrained optimization on six `k_T` and six `Q/T` terms against measured `Fz,Tx,Ty,Tz`. The cost gives yaw torque 1000× weight and penalizes inter-motor coefficient dispersion.

Important uncertainty: lines 131–134 square the **sum of residuals**, rather than summing squared residuals. That allows positive and negative sample errors to cancel and weakens the fit. This is a methodological issue, not merely formatting. The earlier live script computes alternate coefficients, and other scripts hard-code all `1e-7`. Re-identifying the constants with a conventional least-squares cost and held-out validation is strongly recommended.

### Reaction torque, limits, command mapping, and motor dynamics

| Field | Archive result | Classification |
|---|---|---|
| `Q/T` ratio | 0.02 m, common to all motors in final standalone function | Likely; experimentally motivated but optimization result not preserved |
| RPM relationship | `T_i = k_T,i RPM_i²` | Known for AIAA model |
| Paper/test RPM range | 0–10,000 RPM | Known as AIAA experiment range; use as provisional v0.1 clamp, not guaranteed hardware maximum |
| Separate RPM DOE range | data extends to roughly 16,000 RPM for some bench configurations | Known but configuration-dependent; not a flight limit |
| PX4/normalized actuator → RPM | no authoritative function found | Missing |
| PWM/DShot → RPM | calibration data exists for telemetry/tachometer comparison, not a single flight-ready map | Missing/conflicting |
| Motor/ESC lag or time constant | no validated value found for the AIAA/V004 propulsion system | Missing |
| RPM slew/acceleration limit | no authoritative value found | Missing |

For v0.1, command each rotor in RPM (or thrust) directly and validate the static wrench first. Add actuator dynamics only after the bridge interface and step-test data are known.

### AIAA hardware compatibility warning

The AIAA manuscript describes a DJI F550, 9.45-inch props, 960 KV motors, 35 A ESCs, prop guards, Pixhawk 6X, Raspberry Pi 4B, and motion capture. The V004 CAD folders contain `DRONE_Pixracer_V001.prt`, `DRONE_Raspi_Simplified_V001.prt`, `DRONE_Arduino_Mega_V001.prt`, motor/propeller parts, and marker-mount parts. Therefore the 2025 coefficients likely describe the most recent propulsion test hardware, but the 2023 V004 base inertial values cannot automatically be paired with them without updating the assembled mass properties.

Primary paper source: [Approved AIAA 2025 manuscript](../../NG_ASU_KT/AIAA_2025/Manuscript/Approved%2024-2410%20AIAA_2025_Manuscript-11_formatting_fixed.pdf), especially the test article and frame diagram on PDF page 3, processing description on page 5, and nominal allocation/RPM-squared model on page 6.

## Manipulator servo model

The archive confirms legacy **Dynamixel AX-12A** infrastructure:

- 1,000,000 baud is encoded in [AX12A_Util.hpp L6-L12](../../NG_ASU_KT/MECC_2023/Software/Dynamixel_Arduino/src/AX12A_Util.hpp#L6-L12);
- raw goal position, moving speed, torque limit, present position/speed/load, compliance margin/slope, and punch registers are exposed in [AX12A_Util.hpp L24-L77](../../NG_ASU_KT/MECC_2023/Software/Dynamixel_Arduino/src/AX12A_Util.hpp#L24-L77);
- an exploratory test uses raw home 511/512, raw limits 0–1023, maximum speed register 1023, and a 0.01 s integration/test step in [main.cpp](../../NG_ASU_KT/MECC_2023/Software/Dynamixel_Arduino/src/main.cpp);
- `SignalReqs.xlsx` specifies 100 Hz servo command and state exchange as a design requirement;
- step, ramp, and sinusoidal OptiTrack CSV trials exist under `Software/Dynamixel_Arduino/Dynamic_Servo/20221128`.

However, the active test code shown instantiates only servo ID 9, while comments refer to much larger arrays. The archive does not establish that this exact test unit and its settings are the two V004 arm joints. The V003 `servo_test_V001.m` electrical/mechanical state-space constants (`B=3.12e-6`, `J=.001`, `N=254`, `K=3.91e-3`, `L=.05`, `R=5`) are exploratory/assumed and are not linked to identified V004 joint behavior.

| Servo field | Result | Classification |
|---|---|---|
| Model family | AX-12A in legacy controller code | Likely for inherited arm; verify physical servos |
| Command convention | raw goal-position register exists; q-to-register sign/offset absent | Missing for V004 |
| Joint position limits | model uses ±150°; raw code permits 0–1023 | Conflicting/model-vs-device |
| Velocity limit | no V004 physical rad/s value | Missing |
| Torque limit | register exists; no validated N m setting | Missing |
| Update rate | 100 Hz requirement; `dt=0.01` in test code | Likely, not confirmed runtime performance |
| Lag/time constant | experimental CSVs available but no archived identified value | Missing; identifiable from data |
| Damping/friction | no validated V004 joint values | Missing |
| Backlash/deadband | no validated values | Missing |

Use ideal position/torque actuators in v0.1, with conservative ±150° limits. Servo dynamics are not required for the first rigid-body and rotor-wrench validation milestone.

## Gravity, aerodynamics, proximity, and sensors

### Gravity

`[0,0,+9.80665]` m/s² is explicit in every examined V004 RBT builder. This is +Z-down in JJ's coordinates. MuJoCo should retain its conventional `[0,0,-9.80665]` world gravity and use an explicit NED/ENU conversion at interfaces. **Known/high confidence.**

### Aerodynamics and proximity

No nominal translational/rotational drag coefficients were found in the V004 reduced rigid-body tree. V004 co-simulation ports accept an already-aggregated body force/torque. AIAA 2025 estimates proximity-induced residual wrench from measured F/T data after subtracting the nominal RPM-derived wrench; its learned/predictive functions are experiment outputs, not fixed coefficients of the nominal V004 plant.

Accordingly:

- v0.1 aerodynamic body drag: **missing; omit initially**;
- proximity/ground-effect model: **available as separate AIAA data-driven research artifact, not a nominal constant set; omit initially**;
- rotor–rotor and rotor–surface interference: **not parameterized for the primitive nominal model**.

This omission is appropriate for the first hover/kinematics/dynamics milestone and should later become an explicit residual-model or identified-aerodynamics layer.

### Sensors and external measurements

| Sensor/element | Archive evidence | Pose/orientation | v0.1 treatment |
|---|---|---|---|
| Flight controller | V004 CAD: Pixracer; AIAA 2025: Pixhawk 6X | Numeric transform not found | Use bridge/default co-located IMU initially; obtain current CAD/measurements before sensor-correlation work |
| Motion capture | marker-mount CAD and AIAA marker rig | Body↔mocap transform handled in processing, but no single current rigid transform specified | Not needed for v0.1 SITL; later calibrate from rigid-body definition |
| Raspberry Pi | simplified CAD; AIAA RPi 4B | Numeric pose not found | mass contribution should be in base composite inertia |
| Arduino | CAD part | Numeric pose not found | same |
| RPM sensing | MAVROS ESC telemetry entries 1–6, 100 Hz data | motor mapping tied to telemetry order, physical arm labels missing | publish simulated per-motor RPM in confirmed PX4 order |
| F/T sensor (test rig) | AIAA processing | displacement `[0,0,0.075]` m between drone and F/T reference in test processing | test-rig-only; not an onboard sensor |
| GPS/barometer/magnetometer | expected PX4 sensors | no JJ numeric positions/orientations found | use bridge defaults for initial SITL |

Sensor poses do not block v0.1 rigid-body dynamics. They do block high-fidelity estimator and hardware-correlation claims.

## Minimum MuJoCo v0.1 Specification

This section intentionally contains only what is necessary for a free base, two-link arm, and six static rotor wrench sources.

| Required field | Proposed v0.1 value | Status | Action before/after build |
|---|---|---|---|
| MuJoCo world gravity | `[0,0,-9.80665]` m/s² | **Known** | Add explicit adapter to JJ/PX4 NED/FRD convention |
| Base free joint | 6-DOF quaternion free joint | **Known** | Do not reproduce V004 pitch/roll locks |
| Base mass/inertia | EKF values: 1.72557981175 kg; diag(`[.01553695,.05466192,.06896518]`) kg m² | **Conflicting** | Seed model; replace after target-build mass-property measurement |
| Base CoM | body origin | **Likely** as JJ convention, **missing** physically | Seed zero; measure/current-CAD later |
| Link 1 mass/CoM/inertia | EKF values above | **Likely** | Use unless arm hardware differs |
| Link 2 mass/CoM/inertia | EKF short-link values above | **Conflicting** with ACC | Select short link for baseline; make ACC a named future variant |
| Body→J1 transform | +0.36 m X | **Known** for V004 |
| J1→J2 transform | +0.044 m X | **Known** for V004 |
| J2→EE transform | +0.066 m X | **Conflicting** with ACC 0.196 m | Use short baseline |
| Joint axes | J1 +X, J2 +Y | **Known** |
| Joint limits | ±150° each | **Likely** | Adequate model clamp; verify hardware safety limits later |
| Joint damping/friction | 0 initially | **Missing** | Identify later; do not invent |
| Joint actuation | ideal position or torque input | **Missing physical model** | Sufficient for first milestone |
| Rotor hub radius/layout | `l=.275` m and positions listed above | **Likely** for AIAA/current F550 | Confirm physical motor numbering |
| Rotor axes | all body -Z | **Likely/high** |
| Rotor order | AIAA CAM / ESC telemetry order 1–6 | **Known in data**, **missing physically** | Confirm PX4/lab-bridge mapping |
| Rotor spin/yaw signs | `+ - + - + -` reaction moment | **Known** for CAM | Preserve signs; assign CW/CCW labels after inspection |
| Per-motor `k_T` | final standalone AIAA vector | **Conflicting** | Use as versioned nominal set, rerun identification later |
| `Q/T` | 0.02 m | **Likely** | Keep configurable |
| Rotor input | RPM; `T=k_T RPM²` | **Known** | Decouples static model validation from bridge command mapping |
| RPM range | 0–10,000 RPM provisional | **Likely** for AIAA data envelope | Confirm lab bridge/hardware limits |
| Motor lag | none in v0.1 | **Missing** | Add after step-response identification |
| Body aero/proximity | none in v0.1 | **Missing by design** | Add only after nominal validation |
| Sensor sites | body origin/default bridge locations | **Missing physically** | Not a v0.1 dynamics blocker |

### Do we have enough information?

**Yes, for v0.1 and the first PX4-controlled simulated hover.** The model must be labeled a hybrid seed: V004 EKF/WO rigid-body arm plus AIAA 2025 propulsion. It is sufficiently specified to validate topology, signs, frames, kinematics, mass matrix, gravity, rotor allocation, SITL integration, and controller behavior.

**No, for a validated present-hardware digital twin.** That requires the four configuration/mapping questions in the executive verdict, followed by a current assembled mass-property update and actuator identification.

## Validation references

### `FK(q)`

Preferred references:

1. `getTransform(rbtForCodegen(), q, 'L9')` or `getTransform(UAM, q, 'L9')` from the selected V004 branch.
2. [V003 `FK_func.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV003/FK_func.m#L1-L21) as an independent analytical oracle, after explicitly substituting V004 lengths (0.36, 0.044, 0.066). Do not compare it unchanged and mistake millimeter-scale geometry differences for a MuJoCo error.

Avoid `SymbolicForwardKinematics.m` and the existing kinematics checker until their transform/sample issues are corrected.

### `M(q)` and `G(q)`

Preferred references:

- `massMatrix(rbtForCodegen(), q)`;
- `gravityTorque(rbtForCodegen(), q)`;
- the selected branch's `UAM_RBT.mat` loaded into Robotics System Toolbox;
- [ODE_45_forward_dynamics.m](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/ODE_45_forward_dynamics.m) for JJ's force-balance convention.

The V003 generated `B_func.m` is suspicious because it reshapes the result as 6×6 while the surrounding model has eight generalized coordinates. Do not use it as the primary oracle without regenerating it from `test.m`.

### `W_rotors(u)`

Preferred reference:

- [AIAA 2025 `wrench_from_RPM.m`](../../NG_ASU_KT/AIAA_2025/RPM_FT_Data/wrench_from_RPM.m#L1-L24), with input explicitly defined as RPM and output `[Fz,Tx,Ty,Tz]` in the FRD body frame.

The paper's equation and the code agree on the CAM structure. Use the code as the executable reference and preserve the coefficient set/version in test metadata.

## Recommended regression cases

Use tolerances separately for position, rotation, mass matrix, generalized gravity, and wrench. Run every rigid-body test against both the selected MATLAB tree and MuJoCo after converting coordinates.

### Kinematics

1. **Zero:** `q=[0,0,0,0,0,0,0,0]`. Standard V004 EE position in body is `[0.470,0,0]` m; ACC is `[0.600,0,0]` m.
2. **Base transform only:** `[1,-2,0.5, pi/2,0,0, 0,0]`. Confirms translation, yaw order, and frame conversion.
3. **J1 isolation:** `[0,0,0,0,0,0, pi/2,0]`. Confirms local +X joint sign and EE orientation.
4. **J2 isolation:** `[0,0,0,0,0,0, 0,pi/2]`. Standard EE position should be approximately `[0.404,0,-0.066]` m in JJ's body convention; ACC `[0.404,0,-0.196]` m.
5. **Mixed:** `[0.2,-0.1,0.7, 30°,-10°,15°, 45°,-30°]`. Detects rotation-order and parent/child transform errors.

### Mass and gravity

1. zero pose and zero rates;
2. pitch ±30° with arm zero;
3. `q1=±60°`, `q2=±45°` combinations;
4. at least 100 deterministic random valid poses.

For each pose, check that `M` is symmetric and positive definite, compare all elements, and compare `G` with MuJoCo inverse dynamics at zero velocity/acceleration. Also finite-difference MuJoCo potential energy as an independent gravity check.

### Rotor wrench

1. all RPM zero → exact zero wrench;
2. one rotor at 5,000 RPM, others zero → compare each CAM column;
3. all rotors at 5,000 RPM → expected `[Fz,Tx,Ty,Tz] = [-15.0, -0.034375, 0.208387363, -0.0025]` in N/N m using the final standalone coefficients;
4. all at 10,000 RPM → `[-60.0, -0.1375, 0.833549451, -0.01]`;
5. equal `+` yaw-sign group (1,3,5) versus equal `-` group (2,4,6) to verify reaction-torque signs;
6. a deterministic unequal vector such as `[3000,4000,5000,6000,7000,8000]` RPM.

The nonzero lateral moments in equal-RPM cases are expected from JJ's unequal per-motor coefficients. A controller may need unequal hover RPMs; do not “fix” the test by forcing equal coefficients.

## Immediate physical checks and measurements

Before calling v0.1 a current-hardware twin, perform the following minimal checks:

1. photograph and label the aircraft top view with body +X/+Y, motors 1–6, prop CW/CCW, and PX4 output numbers;
2. record the exact base/arm/tool configuration and avionics/guard payload;
3. weigh the complete base and each detachable arm/tool assembly;
4. obtain current NX assembly mass properties in one declared body coordinate system (or measure CoM/inertia experimentally);
5. measure zero-pose joint centers and EE point;
6. record servo models, IDs, raw zero positions, signs, software/hard limits, and supply voltage;
7. inspect the lab MuJoCo bridge actuator order, command units, saturation, and motor lag implementation;
8. rerun the 2025 RPM/F/T coefficient fit with sum-of-squared residuals and held-out runs.

These checks are small compared with a CAD-perfect reconstruction and directly resolve the uncertainties that matter to dynamics and PX4 integration.

## Source map

The files most useful for reproducing or reviewing this audit are:

- standard V004 tree definition: [EKF/WO `UAM_RBT_Builder.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/UAM_RBT_Dev/UAM_RBT_Builder.m);
- generated standard V004 tree: [EKF/WO `rbtForCodegen.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/rbtForCodegen.m);
- extended ACC tree definition: [ACC `UAM_RBT_Builder.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_ACC_2024/UAM_RBT_Dev/UAM_RBT_Builder.m);
- standard forward dynamics: [EKF/WO `ODE_45_forward_dynamics.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/ODE_45_forward_dynamics.m);
- independent V003 kinematics: [V003 `FK_func.m`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV003/FK_func.m);
- co-simulation signal contract: [V004 `20230605_ports.xlsx`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/20230605_ports.xlsx), Sheet1 (outputs A1:A39 and inputs B1:B8);
- interface rate requirements: [V004 `SignalReqs.xlsx`](../../NG_ASU_KT/MECC_2023/AerialManipulatorV004_EKF_WO/SignalReqs.xlsx), requirements table (100 Hz companion/mocap/servo/RPM signals and 1,000 Hz floating-base wrench);
- final AIAA nominal rotor model: [AIAA `wrench_from_RPM.m`](../../NG_ASU_KT/AIAA_2025/RPM_FT_Data/wrench_from_RPM.m);
- AIAA coefficient optimization: [AIAA `compute_constants.m`](../../NG_ASU_KT/AIAA_2025/RPM_FT_Data/compute_constants.m);
- AIAA experiment and convention reference: [approved manuscript](../../NG_ASU_KT/AIAA_2025/Manuscript/Approved%2024-2410%20AIAA_2025_Manuscript-11_formatting_fixed.pdf);
- legacy servo interface: [AX-12A source](../../NG_ASU_KT/MECC_2023/Software/Dynamixel_Arduino/src/main.cpp) and [register map](../../NG_ASU_KT/MECC_2023/Software/Dynamixel_Arduino/src/AX12A_Util.hpp).
