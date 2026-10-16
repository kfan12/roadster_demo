// landing.hpp - landing brake burn (plan 5.6 / 5.8).
//
// Constant-deceleration velocity profile down to a touchdown speed:
//   v_ref(h) = sqrt(v_td^2 + 2 a_b h)      (descent speed, positive down)
// The car free-falls until it is descending faster than v_ref(h), then the
// brake latches on and commands an upward world force
//   F_up = m (g + a_b + k_v (v_down - v_ref(h)))
// clamped to [0, F_max]. On the profile the feed-forward m (g + a_b) alone
// holds the car on it; the feedback absorbs thrust lag and model error.
#pragma once

#include <algorithm>
#include <cmath>

namespace roadster_control
{

struct LandingParams
{
  double brake_accel = 14.7;     // a_b [m/s^2], 1.5 g
  double touchdown_speed = 1.5;  // v_td [m/s]
  double k_v = 3.0;              // [1/s]
  double gravity = 9.81;
};

class LandingBrake
{
public:
  explicit LandingBrake(const LandingParams & p)
  : p_(p) {}

  double profileSpeed(double h) const
  {
    return std::sqrt(p_.touchdown_speed * p_.touchdown_speed +
             2.0 * p_.brake_accel * std::max(h, 0.0));
  }

  // h: COM height above its landed height [m]; vz: world vertical velocity
  // (negative = descending). Returns the commanded upward world force [N].
  double upForce(double h, double vz, double mass, double f_max)
  {
    const double v_down = -vz;
    const double err = v_down - profileSpeed(h);
    if (!active_ && err >= 0.0) {
      active_ = true;
    }
    if (!active_) {
      return 0.0;
    }
    const double f = mass * (p_.gravity + p_.brake_accel + p_.k_v * err);
    return std::clamp(f, 0.0, f_max);
  }

  bool active() const {return active_;}
  void reset() {active_ = false;}

private:
  LandingParams p_;
  bool active_ = false;
};

}  // namespace roadster_control
