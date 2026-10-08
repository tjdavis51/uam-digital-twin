# Controller workbench

This milestone is numerical software only. It does not publish commands, arm a
vehicle, load experiment YAML, or claim closed-loop validation. The three user
algorithms intentionally return `not_implemented` with no value.

## Build and test without ROS

Requirements: CMake >= 3.16 and a C++17 compiler. No Python packages, Eigen,
GoogleTest, PX4, Gazebo, ROS, network downloads, or simulator are needed.
From the repository root:

```bash
./scripts/test_workbench.sh
```

Expected initial result: `core_contract` passes; `acceptance_pd`,
`acceptance_hold`, and `acceptance_smooth` are explicitly skipped. CTest's
"100% tests passed" does NOT mean the skipped algorithms work.

To require every algorithm to be implemented and correct:

```bash
./scripts/test_workbench.sh -DUAM_REQUIRE_ALGORITHMS=ON
```

This deliberately fails until all three algorithms are implemented. The normal
script explicitly restores the default skip behavior on the next invocation.
Run a single test while working:

```bash
ctest --test-dir build/workbench -R '^acceptance_pd$' --output-on-failure
```

## Your implementation order

Open `src/uam_core/src/core.cpp` and find `USER IMPLEMENTATION 1`.
Implement ONLY `PdController::compute_impl` first:

1. Create a `PdOutput`.
2. Compute reference-minus-estimate position and velocity errors.
3. Populate feedforward, proportional and velocity-feedback contributions using
   `config_.kp` and `config_.kv`, independently for each axis.
4. Sum the three contributions into raw acceleration. Do not add gravity,
   integrate an error, numerically differentiate position, or limit commands here.
5. Copy reference yaw and yaw rate into the output.
6. Return `{Status::ok, output}`.

For gains (2,3,4) and (.5,1,1.5), errors (1,-2,.5) and (-.4,.2,0), and
feedforward (.1,0,-.3), the expected acceleration is (1.9,-5.8,1.7).
The test also checks each contribution, zero-error feedforward, zero hold,
yaw passthrough, and axis independence.

Then implement `HoldTrajectory::sample_impl`: constant configured position and
yaw, with analytic zero velocity, acceleration, and yaw rate.

Finally implement `SmoothAxisTrajectory::sample_impl`: derive a quintic scalar
progress polynomial from six constraints (start/end position and zero start/end
velocity and acceleration). Evaluate analytic derivatives with the appropriate
duration scaling. Apply displacement along the configured axis; other coordinates
remain fixed. Keep configured yaw constant. For `t >= duration_s`, return the
final hold. Negative or nonfinite time is rejected by the existing wrapper.
Axis 2 is the same trajectory mathematics for vertical motion.

The tests cover all axes, positive/negative/zero displacement, endpoint holds,
midpoint values, and derivative consistency. Finite differences appear only in
tests, never as the generator implementation.

The public wrappers already reject nonfinite inputs and malformed results.
Constructors reject invalid configurations. An unsuccessful result never exposes
a value. Callers must check both status and value; never use `value_or({})` to
manufacture a zero command. The core has no timestamp, estimator-health, expiry,
or limiter implementation: those belong to future runtime wrappers.

## ROS workspace integration (Ubuntu VM)

Keep the existing PX4 and support-workspace builds. Build our separate overlay:

```bash
./scripts/build_project.sh
source install/project/setup.bash
colcon --log-base log/project test --build-base build/project
colcon test-result --test-result-base build/project --verbose
ros2 interface show uam_interfaces/msg/TrajectoryReference
```

`uam_core` uses the colcon `cmake` build type and can also build standalone.
`uam_interfaces` uses `ament_cmake`/rosidl. ROS message generation must be tested
in the Jazzy environment; a standalone macOS build does not verify it.
The library exports the CMake target `uam_core::uam_core` via
`find_package(uam_core CONFIG REQUIRED)` after installation.

## Configuration and scope

`experiments/workbench.example.yaml` establishes backend-independent configuration
names and units. It is a design template, not executable configuration. C++
configuration validation is implemented; YAML loading is deferred to runtime
integration. Numerical example gains are not aircraft defaults. Unset runtime
limits/timeouts must block future runtime activation rather than imply infinity.

No ROS wrappers, launch nodes, command limiter, frame converter, PX4 adapter,
experiment runner, or automatic arming is implemented yet. These are subsequent
milestones, after you complete and understand the numerical core.

Package manifests currently reserve rights (`LicenseRef-Proprietary`) because the
repository has no chosen public license. This does not establish a release license;
choose the actual license before distributing packages.

## Scaffolding verification (2026-09-29)

Verified locally on macOS with AppleClang 21 and CMake:
standalone Debug build, contract tests, explicit skipped algorithm tests, strict
acceptance failure for all three unfinished algorithms, installation and a separate
CMake consumer linking the exported target. Shell syntax and package XML parsing
also passed. ROS/rosidl generation and colcon integration remain unverified until
run in the Ubuntu/Jazzy VM. No simulator or physical aircraft was commanded.
