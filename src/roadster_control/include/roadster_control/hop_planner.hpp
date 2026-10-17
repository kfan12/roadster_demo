// hop_planner.hpp - Demo B, the obstacle hop: plan the lift burn (plan 5.8).
//
// The car approaches on its wheels at speed v along +x and lifts off on its
// bottom thrusters with a constant net upward acceleration a for a time T,
// then flies at the same v (no drag, the rear nozzles stay shut), optionally
// rolling flip_turns turns in a minimum-jerk profile over flip_duration from
// the burn end. Every obstacle i is a box spanning [x0, x1] along the path
// with top z_top; the car is over it while its COM is within the car's half
// length of the box, and must keep its lowest point `clearance` above the top
// there. Upright the lowest point is the hanging wheels (bottom_below_com
// under the COM); rolled by phi it is the deepest corner of the car's
// cross-section (+-half_width, -bottom_below_com .. +roof_above_com).
//
// The planner flies the plan: it integrates the COM height (burn; then the
// roll's own thrust, which with no roof nozzles comes from one side's bottom
// pair, |tau| / roll_lever along body z, and lifts because the torque peaks
// while the car is nearly upright; gravity) and the roll angle, for a burn
// time T. For that trajectory it finds the burn start x_s that maximises the
// smallest clearance over the row, and it bisects T for the shortest burn
// whose best clearance is `clearance`. A closed form (an apex parabola plus
// the roll's lift as one kick at the burn end) cleared one box by 0.65 m
// instead of 1.0 m when flown with the lift spread over the roll.
// The integration continues down to where the car meets a constant brake_accel
// landing profile, v_down = sqrt(2 a_b h): the landing burn's latch, which the
// flip must end before, or the plan is rejected.
//
// Impulse estimates: up m (g + a) T; flip sum |F| dt; landing m v_l +
// m g v_l / a_b with v_l the descent speed at the latch.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace roadster_control
{

struct HopObstacle
{
  double x0, x1;  // extent along the path [m]
  double top;     // top height above the ground [m]
};

struct HopParams
{
  double speed = 15.0;            // v, approach and flight speed [m/s]
  double burn_accel = 14.7;       // a, net upward acceleration of the lift burn [m/s^2]
  double clearance = 1.0;         // over every obstacle [m]
  double half_length = 2.3;       // car, COM to the bumpers [m]
  double bottom_below_com = 0.40; // lowest point upright (hanging wheels) under the COM [m]
  double com_rest_height = 0.33;  // COM above the ground standing [m]
  double brake_accel = 14.7;      // landing burn profile, for the latch and impulse [m/s^2]
  double gravity = 9.81;
  // flip (0 turns: none)
  double flip_turns = 0.0;
  double flip_duration = 1.4;     // [s]
  double roll_inertia = 597.9;    // [kg m^2]
  double roll_lever = 0.75;       // bottom nozzle y offset [m]
  double half_width = 0.93;       // cross-section corners: wheels' outer faces [m]
  double roof_above_com = 0.93;   // cabin roof above the COM [m]
};

struct HopPlan
{
  bool ok = false;
  double clearance = 0.0;      // smallest planned clearance over the row [m]
  double apex_x = 0.0;         // [m]
  double apex_height = 0.0;    // COM above its standing height [m]
  double apex_time = 0.0;      // after the burn start [s]
  double burn_time = 0.0;      // T [s]
  double burn_vz = 0.0;        // a T, vertical speed at burn end [m/s]
  double burn_start_x = 0.0;   // COM x where the burn starts [m]
  double up_impulse = 0.0;     // [N.s]
  double landing_impulse = 0.0;  // [N.s]
  double flip_impulse = 0.0;   // [N.s]
  double flip_end_time = 0.0;  // after the burn start [s]
  double brake_latch_time = 0.0;  // after the burn start [s]
  double max_depth = 0.0;      // deepest point under the COM over any obstacle [m]
  std::vector<double> clearances;  // per obstacle [m]
};

inline double hopMinJerk(double u, double * dds = nullptr)
{
  u = std::clamp(u, 0.0, 1.0);
  if (dds) {
    *dds = 60 * u - 180 * u * u + 120 * u * u * u;
  }
  return u * u * u * (10 - 15 * u + 6 * u * u);
}

// deepest point of the car's cross-section under the COM, rolled by phi
inline double hopRollDepth(const HopParams & p, double phi)
{
  double d = -std::numeric_limits<double>::infinity();
  for (double y : {-p.half_width, p.half_width}) {
    for (double z : {-p.bottom_below_com, p.roof_above_com}) {
      d = std::max(d, -(y * std::sin(phi) + z * std::cos(phi)));
    }
  }
  return d;
}

inline double hopRequiredHeight(const HopObstacle & o, const HopParams & p)
{
  return o.top + p.clearance + p.bottom_below_com - p.com_rest_height;
}

namespace hop_detail
{
constexpr double kDt = 0.001;

struct Trajectory
{
  std::vector<double> low;  // lowest point above the ground, per kDt sample [m]
  double apex_height = 0.0, apex_time = 0.0, latch_time = 0.0, latch_speed = 0.0;
};

inline Trajectory fly(const HopParams & p, double mass, double T)
{
  Trajectory tr;
  const double theta = 2.0 * M_PI * p.flip_turns, Tf = p.flip_duration;
  const bool flip = p.flip_turns > 0.0 && Tf > 0.0;
  double h = 0.0, vz = 0.0;
  for (int k = 0; k < 20000; ++k) {
    const double t = k * kDt;
    // the step's mean acceleration: the burn ends within a step (T is not on
    // the 1 ms grid; rounding it cost ~1 cm of apex), the roll at its midpoint
    const double burn = std::clamp((T - t) / kDt, 0.0, 1.0);
    double az = burn * p.burn_accel - (1.0 - burn) * p.gravity, phi = 0.0;
    if (flip && t + kDt > T) {
      double dds = 0.0;
      const double u = (std::max(t, T) + 0.5 * (t + kDt - std::max(t, T)) - T) / Tf;
      phi = theta * hopMinJerk((t - T) / Tf);
      hopMinJerk(u, &dds);
      if (u < 1.0) {
        az += (1.0 - burn) * std::abs(p.roll_inertia * theta * dds / (Tf * Tf)) /
          p.roll_lever * std::cos(theta * hopMinJerk(u)) / mass;
      }
    }
    tr.low.push_back(h + p.com_rest_height - hopRollDepth(p, phi));
    if (h > tr.apex_height) {
      tr.apex_height = h;
      tr.apex_time = t;
    }
    if (t > T && vz < 0.0 && -vz >= std::sqrt(2.0 * p.brake_accel * std::max(h, 0.0))) {
      tr.latch_time = t;
      tr.latch_speed = -vz;
      break;
    }
    h += vz * kDt + 0.5 * az * kDt * kDt;  // exact for a constant acceleration
    vz += az * kDt;
  }
  return tr;
}

// smallest clearance over the row with the burn starting at x_s; per obstacle too
inline double clearance(const std::vector<HopObstacle> & obs, const HopParams & p,
  const Trajectory & tr, double x_s, std::vector<double> * each = nullptr)
{
  const int n = static_cast<int>(tr.low.size());
  double worst = std::numeric_limits<double>::infinity();
  if (each) {
    each->assign(obs.size(), 0.0);
  }
  for (std::size_t i = 0; i < obs.size(); ++i) {
    const int k0 = static_cast<int>(std::ceil((obs[i].x0 - p.half_length - x_s) / p.speed / kDt));
    const int k1 = static_cast<int>(std::floor((obs[i].x1 + p.half_length - x_s) / p.speed / kDt));
    double c = std::numeric_limits<double>::infinity();
    if (k0 < 0 || k1 >= n) {
      c = -std::numeric_limits<double>::infinity();  // over it on the ground, or after the latch
    } else {
      for (int k = k0; k <= k1; ++k) {
        c = std::min(c, tr.low[k] - obs[i].top);
      }
    }
    if (each) {
      (*each)[i] = c;
    }
    worst = std::min(worst, c);
  }
  return worst;
}

// the burn start with the most clearance: coarse scan, then fine around it
inline double bestStart(const std::vector<HopObstacle> & obs, const HopParams & p,
  const Trajectory & tr, double lo, double * best_c)
{
  double best_x = lo, best = -std::numeric_limits<double>::infinity();
  const double span = tr.low.size() * kDt * p.speed;
  for (double x = lo - span; x <= lo; x += 0.1) {
    const double c = clearance(obs, p, tr, x);
    if (c > best) {
      best = c;
      best_x = x;
    }
  }
  const double x0 = best_x;
  for (double x = x0 - 0.1; x <= x0 + 0.1; x += 0.002) {
    const double c = clearance(obs, p, tr, x);
    if (c > best) {
      best = c;
      best_x = x;
    }
  }
  *best_c = best;
  return best_x;
}
}  // namespace hop_detail

inline HopPlan planHop(const std::vector<HopObstacle> & obs, const HopParams & p, double mass)
{
  using namespace hop_detail;
  HopPlan plan;
  if (obs.empty() || p.speed <= 0.0 || p.burn_accel <= 0.0) {
    return plan;
  }
  double lo = std::numeric_limits<double>::infinity();
  for (const auto & o : obs) {
    lo = std::min(lo, o.x0);
  }
  // shortest burn whose best clearance reaches the target (clearance grows with T)
  double T_lo = 0.0, T_hi = 2.0, c = 0.0;
  bestStart(obs, p, fly(p, mass, T_hi), lo, &c);
  if (c < p.clearance) {
    return plan;  // not even a 2 s burn clears the row
  }
  for (int it = 0; it < 30 && T_hi - T_lo > 1e-4; ++it) {
    const double T = 0.5 * (T_lo + T_hi);
    bestStart(obs, p, fly(p, mass, T), lo, &c);
    (c >= p.clearance ? T_hi : T_lo) = T;
  }
  const double T = T_hi;
  const Trajectory tr = fly(p, mass, T);
  plan.burn_start_x = bestStart(obs, p, tr, lo, &plan.clearance);
  clearance(obs, p, tr, plan.burn_start_x, &plan.clearances);
  plan.burn_time = T;
  plan.burn_vz = p.burn_accel * T;
  plan.apex_height = tr.apex_height;
  plan.apex_time = tr.apex_time;
  plan.apex_x = plan.burn_start_x + p.speed * tr.apex_time;
  plan.brake_latch_time = tr.latch_time;
  const double g = p.gravity, theta = 2.0 * M_PI * p.flip_turns, Tf = p.flip_duration;
  const bool flip = p.flip_turns > 0.0 && Tf > 0.0;
  // deepest point while over any obstacle, and the flip's thrust impulse
  plan.max_depth = p.bottom_below_com;
  for (const auto & o : obs) {
    for (double x = o.x0 - p.half_length; x <= o.x1 + p.half_length; x += 0.05) {
      const double t = (x - plan.burn_start_x) / p.speed;
      plan.max_depth = std::max(plan.max_depth, flip ?
        hopRollDepth(p, theta * hopMinJerk((t - T) / Tf)) : p.bottom_below_com);
    }
  }
  if (flip) {
    const int n = 4000;
    for (int k = 0; k < n; ++k) {
      double dds = 0.0;
      hopMinJerk((k + 0.5) / n, &dds);
      plan.flip_impulse += std::abs(p.roll_inertia * theta * dds / (Tf * Tf)) / p.roll_lever * Tf / n;
    }
    plan.flip_end_time = T + Tf;
  }
  plan.up_impulse = mass * (g + p.burn_accel) * T;
  plan.landing_impulse = mass * tr.latch_speed + mass * g * tr.latch_speed / p.brake_accel;
  plan.ok = tr.latch_time > 0.0 && (!flip || plan.flip_end_time + 0.1 < plan.brake_latch_time);
  return plan;
}

}  // namespace roadster_control
