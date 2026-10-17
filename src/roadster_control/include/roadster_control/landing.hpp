// landing.hpp - landing brake burn (plan 5.6 / 5.8).
//
// Two-stage constant-deceleration velocity profile down to a touchdown speed:
// a_b down to the flare height h_f, then the harder flare a_f to v_td at the
// ground, so the car comes in slowly without a long, gas-hungry slow descent:
//   v_ref(h) = sqrt(v_td^2 + 2 a_f min(h, h_f) + 2 a_b max(h - h_f, 0))
// (descent speed, positive down). The car free-falls until it is descending
// faster than v_ref(h), then the brake latches on and commands an upward
// world force
//   F_up = m (g + a + k_v (v_down - v_ref(h))),  a = a_f below h_f, else a_b
// clamped to [0, F_max]. On the profile the feed-forward m (g + a) alone
// holds the car on it; the feedback absorbs thrust lag and model error.
// h_f = 0 is the single-stage profile.
// The flare needs its own, stiffer feedback (k_vf): the profile's slope in
// time is a v_down / v_ref, so an error e grows at a e / v_ref, faster as
// v_ref falls to v_td; with k_v = 3 a 0.2 m/s entry error grew to 0.9 m/s by
// touchdown (flare_imu1). The flare's feed-forward also starts lead_time early
// (h - v_down lead_time < h_f) so the ~40 ms thrust rise is done at h_f. It
// ends at v_down = v_td / 2 + a t_fall (v_td / 2 is the bounce guard below):
// the thrust decays with the valves' fall lag t_fall and removes about a t_fall
// more speed. Ending at h_td regardless stopped the car 1.5 cm high and it
// hovered 0.2 s (flare_imu3); ending lead_time ahead in height cut the braking
// while still at 1.4 m/s (flare_imu4).
// Heights in the profile are above h_td, the touchdown height: the profile
// reaches v_td there and holds it below (no deceleration feed-forward). In
// flight the suspension hangs extended, so the wheels touch with the COM
// ~2 cm above its standing height, where a profile ending at 0 still asked
// for 1.1 m/s (flare_imu2).
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
  double flare_height = 0.0;     // h_f [m]; 0: no flare stage
  double flare_accel = 14.7;     // a_f [m/s^2]
  double flare_kv = 3.0;         // k_vf [1/s]
  double flare_lead_time = 0.0;  // [s]
  double touchdown_height = 0.0; // h_td [m]
  double thrust_fall_time = 0.0; // t_fall [s], the nozzle valves' fall lag
  double gravity = 9.81;
};

class LandingBrake
{
public:
  explicit LandingBrake(const LandingParams & p)
  : p_(p) {}

  double profileSpeed(double h) const
  {
    const double hs = std::max(h - p_.touchdown_height, 0.0);
    return std::sqrt(p_.touchdown_speed * p_.touchdown_speed +
             2.0 * p_.flare_accel * std::min(hs, p_.flare_height) +
             2.0 * p_.brake_accel * std::max(hs - p_.flare_height, 0.0));
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
    // the deceleration feed-forward only while really coming down: after a
    // bounce (rising) it kept ~0.9 m g of thrust on and the car hovered at
    // 0.9 m until the tank ran dry (drop run est_shadow3, 2026-10-16)
    const double hp = h - p_.touchdown_height;  // height in the profile
    const bool flare = hp - std::max(v_down, 0.0) * p_.flare_lead_time < p_.flare_height;
    const double a = hp <= 0.0 ? 0.0 : (flare ? p_.flare_accel : p_.brake_accel);
    const double a_ff = v_down > 0.5 * p_.touchdown_speed + a * p_.thrust_fall_time ? a : 0.0;
    const double f = mass * (p_.gravity + a_ff + (flare ? p_.flare_kv : p_.k_v) * err);
    return std::clamp(f, 0.0, f_max);
  }

  bool active() const {return active_;}
  void reset() {active_ = false;}

private:
  LandingParams p_;
  bool active_ = false;
};

}  // namespace roadster_control
