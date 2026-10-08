#include "uam_core/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// defining the uam_core namespace
namespace uam_core {

// defining an anonymous namespace for file local helpers
namespace {

// checks if a Vec3 is not given bad or infinite values
bool finite(const Vec3& v) {
  // checks if all of the vecor values are doubles
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}

// checks if any of the values in a trajectory sample are bad
bool finite(const TrajectorySample& r) {
  return finite(r.position) && finite(r.velocity) && finite(r.acceleration)
      && std::isfinite(r.yaw) && std::isfinite(r.yaw_rate);
}

// checks if any values in the PD output are bad
bool finite(const PdOutput& r) {
  return finite(r.position_error) && finite(r.velocity_error) && finite(r.feedforward)
      && finite(r.proportional) && finite(r.velocity_feedback) && finite(r.acceleration)
      && std::isfinite(r.yaw) && std::isfinite(r.yaw_rate);
}

// checks is a result is good
template <typename T> Result<T> checked(Result<T> result) {
  // if not ok then
  if (result.status != Status::ok) {
    // remove the output completely
    result.value.reset();
    
    // is the status is ok, check if either a value does not exist or is not valid 
  } else if (!result.value || !finite(*result.value)) {
    return {Status::invalid_input, std::nullopt};
  }

  // if the tests pass then pass the result along
  return result;
}
}  // end the anonymous namespace

// checks if the PD config is all valid
bool valid(const PdConfig& c) {
  const auto nonnegative = [](double x) { return x >= 0.0; };
  return finite(c.kp) && finite(c.kv)
      && std::all_of(c.kp.begin(), c.kp.end(), nonnegative)
      && std::all_of(c.kv.begin(), c.kv.end(), nonnegative);
}

// checks if a hold config is valid
bool valid(const HoldConfig& c) { return finite(c.position) && std::isfinite(c.yaw); }

// check if a smooth axis config is valid
bool valid(const SmoothAxisConfig& c) {
  return finite(c.start) && c.axis < 3 && std::isfinite(c.displacement)
      && std::isfinite(c.duration_s) && c.duration_s > 0.0 && std::isfinite(c.yaw)
      && std::isfinite(c.start[c.axis] + c.displacement);
}

// constructor definition for PD controller
PdController::PdController(PdConfig c) : config_(c) {
  // if the config is not valid it throws the error with tip
  if (!valid(c)) throw std::invalid_argument("PD gains must be finite and nonnegative");
}

// constructor for the hold trajectory class
HoldTrajectory::HoldTrajectory(HoldConfig c) : config_(c) {
  if (!valid(c)) throw std::invalid_argument("Hold configuration must be finite");
}

// constructor for the smooth axis trajectory class
SmoothAxisTrajectory::SmoothAxisTrajectory(SmoothAxisConfig c) : config_(c) {
  if (!valid(c)) throw std::invalid_argument("Invalid smooth-axis configuration");
}

// public wrapper for the compute method, 
// checks if the input is good then if computation is good
Result<PdOutput> PdController::compute(const TranslationalState& state,
                                      const TrajectorySample& reference) const {
  if (!finite(state.position) || !finite(state.velocity) || !finite(reference))
    return {Status::invalid_input, std::nullopt};
  return checked(compute_impl(state, reference));
}

// public wrapper for the sample method, checks if the input is good
// if so then it runs the sample and checks if the output is good
// runs for the hold trajectory
Result<TrajectorySample> HoldTrajectory::sample(double t) const {
  if (!std::isfinite(t) || t < 0.0) return {Status::invalid_input, std::nullopt};
  return checked(sample_impl(t));
}

// public wrapper for the sample method, checks if the input is good
// if so then it runs the sample and checks if the output is good
// runs for the smooth axis trajectory
Result<TrajectorySample> SmoothAxisTrajectory::sample(double t) const {
  if (!std::isfinite(t) || t < 0.0) return {Status::invalid_input, std::nullopt};
  return checked(sample_impl(t));
}

///////////////////////////////////////////////////////////////////////
// compute errors, three contributions, their sum, and yaw passthrough.
// Return {Status::ok, output} only when your calculation is complete
// does not include gravity
// takes in the state and computes the acceleration for the trajectory
Result<PdOutput> PdController::compute_impl(const TranslationalState& state,
                                           const TrajectorySample& reference) const {

  // takes in the translational state, 
  // which gives access to the vector for postion and the vector for velocities
  // and takes in the trajectory sample
  // which gives postion, velocity, acceleration, yaw, and yaw rate
  
  // I'll just contruct all the parts indivually
  // start with the position error
  // e_p,i = p_d,i - p-hat_i
  Vec3 position_errors{};
  // save the position errors
  for (std::size_t i = 0; i < 3; i++) {
    position_errors[i] = reference.position[i] - state.position[i];
  }

  // next move to the velocity errors
  // e_v,i = v_d,i - v-hat_i
  Vec3 velocity_errors{};
  // save the errors
  for (std::size_t i = 0; i < 3; i++) {
    velocity_errors[i] = reference.velocity[i] - state.velocity[i];
  }

  // feedforward
  Vec3 feedforwards = reference.acceleration;

  // portional
  // k_p,i * e_p,i
  Vec3 proportional{};
  for (std::size_t i = 0; i < 3; i++) {
    proportional[i] = config_.kp[i] * position_errors[i];
  }

  // velocity feedback
  // k_v,i * e_v,i
  Vec3 velocity_feedback{};
  for (std::size_t i = 0; i < 3; i++) {
    velocity_feedback[i] = config_.kv[i] * velocity_errors[i];
  }

  // now save the commanded acceleration, which is just the feedforward + postion and velocity feedback
  // acceration formula
  // a_cmd,i = a_d,i + k_p,i * e_p,i + k_v,i * e_v,i
  Vec3 acceleration_commands{};
  for (std::size_t i = 0; i < 3; i++) {
    acceleration_commands[i] = feedforwards[i] + proportional[i] + velocity_feedback[i];
  }

  // make the output object
  PdOutput output;
  // assign its values
  output.position_error = position_errors;
  output.velocity_error = velocity_errors;
  output.feedforward = feedforwards;
  output.proportional = proportional;
  output.velocity_feedback = velocity_feedback;
  output.acceleration = acceleration_commands;
  output.yaw = reference.yaw;
  output.yaw_rate = reference.yaw_rate;

  // return PdOutput
  // position error, velocity error,
  // feedforward, proportional
  // velocity feedback
  // acceleration
  // yaw, and yaw rate

  // return a Result object with the status and the output
  return Result<PdOutput>{Status::ok, output};
}

// analytic constant position, velocity, acceleration, yaw and yaw rate
// a hold will ask the drone to maintain a fixed position and yaw
// config supplies the position and yaw
Result<TrajectorySample> HoldTrajectory::sample_impl(double elapsed_s) const {
  // during a hold the desired postion is constant
  // because a constant does not change with time, the derivative of position (which is velocity) is going to be zero
  // the derivative of that (acceleration) is also going to be zero

  // the heading will also be constant
  // so the heading rate also will be zero

  // vector with all zeros
  Vec3 zeros{};
  
  // everything needed is already accessible
  TrajectorySample sample = {
    this->config_.position, // position
    zeros, // velocity
    zeros, // acceleration
    this->config_.yaw, // yaw
    0.0 // yaw rate
  };

  // there is not need to use the parameter because the time elasped won't effect the hold
  (void)elapsed_s;

  // return the new sample trajectory
  return {Status::ok, sample};
}

// this method does quintic translation with analytic derivatives
// for t >= duration_s return the final hold, negative time is rejected by the wrapper
// basically: move a specified distance along one axis over T seconds, start and finish at rest
Result<TrajectorySample> SmoothAxisTrajectory::sample_impl(double elapsed_s) const {
  // start the sample trajectory object
  TrajectorySample sample{};
  sample.position = this->config_.start;
  sample.yaw = this->config_.yaw;
  sample.yaw_rate = 0.0; // the deriviative of yaw is going to be zero

  // run the position polynomial while the duration is still going
  if (elapsed_s < this->config_.duration_s) {
    // I'll use the quintic time-scaling polynomial
    // first normalize time elapsed with the total duration
    const double normalized_time = elapsed_s / this->config_.duration_s;
    // now time is on a scale from 0 to 1 during movement

    // generate some powers
    const double tau2 = std::pow(normalized_time, 2);
    const double tau3 = std::pow(normalized_time, 3);
    const double tau4 = std::pow(normalized_time, 4);
    const double tau5 = std::pow(normalized_time, 5);

    // first solve for the desired positions
    // desired_position_i = p_0,i + displacement * s(normalized_time)
    // s(normalized_time = tau) = 10(tau^3) - 15(tau^4) + 6(tau^5)
    const double s = (10 * tau3) + (-15 * tau4) + (6 * tau5);
    // get desired position
    sample.position[this->config_.axis] = this->config_.start[this->config_.axis] + (this->config_.displacement * s);

    // next solve for the desired velocities
    // v_d,i = (D / T) * s'(normalized_time)
    // find s_prime
    const double s_prime = (30 * tau2) + (-60 * tau3) + (30 * tau4);

    // get the desired velocity
    sample.velocity[this->config_.axis] = (this->config_.displacement / this->config_.duration_s) * s_prime;

    // last solve for the desired accelerations
    // a_d,i = (D / T^2) * s''(normalized_time)
    // find s_double_prime
    const double s_double_prime = (60 * normalized_time) + (-180 * tau2) + (120 * tau3);
    // now get the desired acceleration
    sample.acceleration[this->config_.axis] = (this->config_.displacement / (std::pow(this->config_.duration_s, 2))) * s_double_prime;
  }

  // otherwise just hold the trajectory
  else { // elapsed time is greater than or equal to the duration
    Vec3 zeros{};
    
    // the position goes to the endpoint
    sample.position[this->config_.axis] = this->config_.start[this->config_.axis] + this->config_.displacement;
    sample.velocity = zeros; // velocity should be zero
    sample.acceleration = zeros; // acceleration should be zero
    sample.yaw = this->config_.yaw; // the heading stays the same
    sample.yaw_rate = 0.0; // yaw rate stays zero
  }

  return {Status::ok, sample};
}
}  // namespace uam_core
