// nozzle_model.hpp - one cold-gas valve + nozzle (plan section 5.3).
//
// Command u in [0, 1] -> actual valve opening through a first-order lag
// (separate rise/fall time constants) with a minimum on-time: once a valve
// opens it stays at its last nonzero command for at least min_on_time_s, so a
// controller cannot chatter it faster than the hardware would allow.
// Thrust F = u_actual * f_max * (P_feed / P_ref); mass flow = F / v_e.
#pragma once

#include <algorithm>
#include <cmath>

namespace roadster_gz_plugins
{

struct NozzleParams
{
  double f_max_n = 0.0;        // at the reference feed pressure
  double tau_rise_s = 0.02;
  double tau_fall_s = 0.02;
  double min_on_time_s = 0.02;
  double exhaust_velocity = 350.0;  // m/s
};

class NozzleModel
{
public:
  explicit NozzleModel(const NozzleParams & p)
  : p_(p) {}

  // Advance by dt; returns the thrust [N] produced during this step.
  double step(double u_cmd, double p_feed, double p_ref, double t_now, double dt)
  {
    u_cmd = std::clamp(u_cmd, 0.0, 1.0);
    if (u_cmd > kOnThreshold) {
      if (!open_) {
        open_ = true;
        on_until_ = t_now + p_.min_on_time_s;
      }
      last_on_cmd_ = u_cmd;
    } else if (open_ && t_now < on_until_) {
      u_cmd = last_on_cmd_;  // hold through the minimum on-time
    } else {
      open_ = false;
    }
    const double tau = u_cmd > u_ ? p_.tau_rise_s : p_.tau_fall_s;
    u_ += (u_cmd - u_) * (tau > 0 ? 1.0 - std::exp(-dt / tau) : 1.0);
    if (u_ < 1e-6) {
      u_ = 0.0;
    }
    thrust_ = (p_ref > 0 && p_feed > 0) ? u_ * p_.f_max_n * (p_feed / p_ref) : 0.0;
    return thrust_;
  }

  double massFlow() const {return p_.exhaust_velocity > 0 ? thrust_ / p_.exhaust_velocity : 0.0;}
  double thrust() const {return thrust_;}
  double opening() const {return u_;}
  void reset() {u_ = 0; thrust_ = 0; open_ = false; on_until_ = 0; last_on_cmd_ = 0;}
  const NozzleParams & params() const {return p_;}

private:
  static constexpr double kOnThreshold = 1e-3;
  NozzleParams p_;
  double u_ = 0.0;
  double thrust_ = 0.0;
  bool open_ = false;
  double on_until_ = 0.0;
  double last_on_cmd_ = 0.0;
};

}  // namespace roadster_gz_plugins
