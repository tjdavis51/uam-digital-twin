#include "uam_core/core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
using namespace uam_core;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double a, double b, double tolerance = 1e-9) {
  require(std::isfinite(a) && std::abs(a - b) <= tolerance, "numerical mismatch");
}

void near(const Vec3& a, const Vec3& b, double tolerance = 1e-9) {
  for (unsigned i = 0; i < 3; ++i) near(a[i], b[i], tolerance);
}

struct Unimplemented {};

template <typename T> T value(const Result<T>& r) {
  if (r.status == Status::not_implemented) {
    require(!r.value, "unfinished algorithm exposed output");
    throw Unimplemented{};
  }

  require(r.status == Status::ok && r.value.has_value(), "expected valid result");
  return *r.value;
}

void contract() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  require(valid(PdConfig{}), "zero gains allowed for isolated tests");
  require(!valid(PdConfig{{-1, 0, 0}, {}}), "negative gain rejected");
  require(!valid(PdConfig{{}, {0, inf, 0}}), "infinite gain rejected");
  require(!valid(HoldConfig{{nan, 0, 0}, 0}), "invalid hold rejected");
  require(!valid(SmoothAxisConfig{{}, 3, 1, 1, 0}), "invalid axis rejected");
  require(!valid(SmoothAxisConfig{{}, 0, 1, 0, 0}), "zero duration rejected");

  bool threw = false;
  try { PdController bad({{-1, 0, 0}, {}}); } catch (const std::invalid_argument&) { threw = true; }
  require(threw, "invalid configuration must not construct");
  PdController controller({{2, 3, 4}, {.5, 1, 1.5}});
  auto invalid = controller.compute({{nan, 0, 0}, {}}, {});
  require(invalid.status == Status::invalid_input && !invalid.value, "bad state fails closed");
  TrajectorySample reference;
  reference.yaw_rate = inf;
  invalid = controller.compute({}, reference);
  require(invalid.status == Status::invalid_input && !invalid.value, "bad reference fails closed");
  HoldTrajectory hold({});
  SmoothAxisTrajectory smooth({});

  for (double t : {-1.0, nan, inf}) {
    require(hold.sample(t).status == Status::invalid_input && !hold.sample(t).value, "bad hold time");
    require(smooth.sample(t).status == Status::invalid_input && !smooth.sample(t).value, "bad smooth time");
  }

  const auto result = controller.compute({}, {});
  require((result.status == Status::not_implemented && !result.value)
       || (result.status == Status::ok && result.value), "result contract");
}

void pd() {
  PdController c({{2, 3, 4}, {.5, 1, 1.5}});
  TrajectorySample r{{1, -2, .5}, {-.4, .2, 0}, {.1, 0, -.3}, .7, -.2};
  auto out = value(c.compute({}, r));
  near(out.position_error, r.position); near(out.velocity_error, r.velocity);
  near(out.feedforward, r.acceleration); near(out.proportional, {2, -6, 2});
  near(out.velocity_feedback, {-.2, .2, 0}); near(out.acceleration, {1.9, -5.8, 1.7});
  near(out.yaw, .7); near(out.yaw_rate, -.2);
  out = value(c.compute({r.position, r.velocity}, r));
  near(out.acceleration, r.acceleration);
  near(value(c.compute({}, {})).acceleration, {0, 0, 0});
  for (unsigned axis = 0; axis < 3; ++axis) {
    TrajectorySample one; one.position[axis] = 1;
    auto isolated = value(c.compute({}, one));
    for (unsigned i = 0; i < 3; ++i) near(isolated.acceleration[i], i == axis ? 2.0 + axis : 0.0);
  }
}

void hold() {
  HoldTrajectory h({{1, -2, 3}, .4});
  for (double t : {0.0, 1.0, 100.0}) {
    auto r = value(h.sample(t));
    near(r.position, {1, -2, 3}); near(r.velocity, {}); near(r.acceleration, {});
    near(r.yaw, .4); near(r.yaw_rate, 0);
  }
}

void smooth() {
  for (unsigned axis = 0; axis < 3; ++axis) {
    for (double displacement : {-2.0, 0.0, 2.0}) {
      SmoothAxisTrajectory trajectory({{1, 2, 3}, axis, displacement, 4, .3});
      auto start = value(trajectory.sample(0));
      near(start.position, {1, 2, 3}); near(start.velocity, {}); near(start.acceleration, {});
      Vec3 endpoint{1, 2, 3}; endpoint[axis] += displacement;
      for (double t : {4.0, 5.0}) {
        auto end = value(trajectory.sample(t));
        near(end.position, endpoint); near(end.velocity, {}); near(end.acceleration, {});
      }

      auto middle = value(trajectory.sample(2));
      Vec3 midpoint{1, 2, 3}; midpoint[axis] += displacement / 2;
      Vec3 speed{}; speed[axis] = 1.875 * displacement / 4;
      near(middle.position, midpoint); near(middle.velocity, speed); near(middle.acceleration, {});
      near(middle.yaw, .3); near(middle.yaw_rate, 0);
      // Finite differences only cross-check the analytic implementation in this test.
      
      const double dt = 1e-4;

      for (double t : {.4, 1.3, 3.7}) {
        auto a = value(trajectory.sample(t - dt));
        auto b = value(trajectory.sample(t));
        auto c = value(trajectory.sample(t + dt));
        for (unsigned i = 0; i < 3; ++i) {
          near((c.position[i] - a.position[i]) / (2 * dt), b.velocity[i], 1e-6);
          near((c.velocity[i] - a.velocity[i]) / (2 * dt), b.acceleration[i], 1e-6);
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "expected test name");
    std::string test = argv[1];
    if (test == "contract") contract();
    else if (test == "pd") pd();
    else if (test == "hold") hold();
    else if (test == "smooth") smooth();
    else throw std::runtime_error("unknown test");
    std::cout << "PASS: " << test << '\n'; return 0;
  } catch (const Unimplemented&) {
    std::cout << "NOT IMPLEMENTED: user algorithm has no valid output yet\n"; return 77;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
}
