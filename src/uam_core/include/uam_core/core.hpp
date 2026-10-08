#pragma once

#include <array>
#include <optional>

// define the namespace for uam_core
namespace uam_core {

// make an alias for a vector of length 3 of doubles
using Vec3 = std::array<double, 3>;

// define the types of Status
enum class Status { ok, invalid_input, invalid_configuration, not_implemented };

// make a Result struct, it can have a status and an optional value associated
// No value means no command/reference is available. Callers must never substitute zeros.
template <typename T> struct Result {
  Status status{Status::not_implemented};
  std::optional<T> value{};
};

// snapshot of the drone state, this defines the position x, y, z
// and the velocity v_x, v_y, v_z
// compiler initializes all values to 0's
// used to tell where the drone is and where is it going
struct TranslationalState {
  Vec3 position{};  // odom, meters
  Vec3 velocity{};  // odom, meters/second
};

// this struct defines more fully a snapshot of the drone,
// the use case will be for comparing against the translational state,
// basically where it should be, where it should be moving, and with what
// desired acceleration, and heading
// there is not need for pitch and roll, px4 will determine that
struct TrajectorySample {
  Vec3 position{};
  Vec3 velocity{};
  Vec3 acceleration{};  // inertial acceleration, no added gravity
  double yaw{0.0};
  double yaw_rate{0.0};
};

// holds the gains for how strongly the conroller will correct position and velocity errors
struct PdConfig {
  // position error gain
  Vec3 kp{};  // 1/s^2, nonnegative
  // velocity error gain
  Vec3 kv{};  // 1/s, nonnegative
};

// will hold the output of 1 PD controller calculation
// this will include the acceleration request and the intermediate terms to explain it
// compute_impl() will be incharge of populating this struct
struct PdOutput {
  Vec3 position_error{}; // desired postion - estimated position
  Vec3 velocity_error{}; // desired velocity - estimated velocity
  Vec3 feedforward{}; // desired trajectory acceleration
  Vec3 proportional{}; // postion error * postion gains
  Vec3 velocity_feedback{}; // velocity error * velocity gains
  Vec3 acceleration{};  // sum of the three acceleration contributions
  double yaw{0.0}; // desired yaw copied from the reference
  double yaw_rate{0.0}; // desired yaw rate compied from the reference
};

// holds the settings for a trajectory
// can go into the hold trajectory function
struct HoldConfig { 
  Vec3 position{}; // desired postion
  double yaw{0.0}; // desired heading 
};

// holds the settings for a trajectory that moves smoothly along one
// world-frame axis, starting and ending at rest
struct SmoothAxisConfig {
  Vec3 start{}; // desired starting position in meters
  unsigned axis{0};  // 0=x, 1=y, 2=z, which coordinate changes
  double displacement{0.0}; // distance to travel along that axis in meters
  double duration_s{1.0}; // time allowed for the movement
  double yaw{0.0}; // heading to maintain throughout the movement in radians
};

// function definitions

// checks for a valid PdConfig
bool valid(const PdConfig& config);

// checks for a valid HoldConfig
bool valid(const HoldConfig& config);

// checks for a valid SmoothAxisConfig
bool valid(const SmoothAxisConfig& config);

// start the class for the PdController
class PdController {

 // public interface
 public:
  // declare the constructor to take in a PdConfig, which is just the set of gains
  explicit PdController(PdConfig config);

  // method for computing the PdOutput from the translational state and the trajectory
  // this is the public facing side accessible
  Result<PdOutput> compute(const TranslationalState& state,
                           const TrajectorySample& reference) const;

 // private interface
 private:
  // here will be the actual implementation of the PD math
  // this method is hidden
  Result<PdOutput> compute_impl(const TranslationalState& state,
                                const TrajectorySample& reference) const;

  // stores the gains for the given controller
  PdConfig config_;
};

// class for maintaining one desired position
class HoldTrajectory {

 public:
  // constructor taking in the config
  explicit HoldTrajectory(HoldConfig config);

  // public interface for taking in a trajectory and sending it on it
  Result<TrajectorySample> sample(double elapsed_s) const;

 private:
  // the actual implementation for holding the drone on a trajectory
  Result<TrajectorySample> sample_impl(double elapsed_s) const;

  // stores the input config
  HoldConfig config_;
};

// class for moving between positions on 1 axis
class SmoothAxisTrajectory {
 public:
  // moves to a new position, starting from zero velocity, varies, then back to zero
  
  // constructor
  explicit SmoothAxisTrajectory(SmoothAxisConfig config);

  // runs the desired trajectory
  Result<TrajectorySample> sample(double elapsed_s) const;
 private:

  // actual implementation, hidden
  Result<TrajectorySample> sample_impl(double elapsed_s) const;

  // member to save the given config
  SmoothAxisConfig config_;
};

}  // end namespace uam_core
