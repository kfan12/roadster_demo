// tank_model.hpp - compressed-air tank for the roadster cold-gas thrusters.
//
// Isothermal real-gas reservoir (docs/plans/roadster_demo_plan.md 5.2). The
// compressibility of air at ~400 bar is not negligible (Z ~ 1.1-1.15), so the
// equation of state is P = Z(P) rho R T with the linear fit Z = 1 + k P,
// which inverts in closed form: P = rho R T / (1 - k rho R T).
// Feed modes: REGULATED (nozzles see p_reg while P_tank > p_reg, blowdown
// below it) or BLOWDOWN (nozzles see P_tank throughout). Below p_min nothing
// fires. Pure math, no Gazebo types, unit-tested in test/test_models.cpp.
#pragma once

#include <algorithm>
#include <stdexcept>

namespace roadster_gz_plugins
{

struct TankParams
{
  double volume_m3 = 0.6;
  double p0_pa = 400e5;
  double temperature_k = 293.0;
  double p_reg_pa = 100e5;
  double p_min_pa = 25e5;
  double z_slope_per_pa = 0.35e-8;  // Z(400 bar) = 1.14
  bool regulated = true;
};

class TankModel
{
public:
  static constexpr double kRAir = 287.05;  // J/(kg K)

  explicit TankModel(const TankParams & p)
  : p_(p)
  {
    if (p_.volume_m3 <= 0 || p_.temperature_k <= 0 || p_.p0_pa <= 0) {
      throw std::invalid_argument("tank: volume, temperature and p0 must be > 0");
    }
    recharge();
  }

  // Gas mass for a given pressure (inverse of pressure()).
  double massAt(double p_pa) const
  {
    const double z = 1.0 + p_.z_slope_per_pa * p_pa;
    return p_pa * p_.volume_m3 / (z * kRAir * p_.temperature_k);
  }

  double pressure() const
  {
    const double rrt = (mass_ / p_.volume_m3) * kRAir * p_.temperature_k;
    return rrt / (1.0 - p_.z_slope_per_pa * rrt);
  }

  // Pressure the nozzles see; 0 when the tank is below the firing floor.
  double feedPressure() const
  {
    const double pt = pressure();
    if (pt < p_.p_min_pa) {
      return 0.0;
    }
    return p_.regulated ? std::min(pt, p_.p_reg_pa) : pt;
  }

  // Nominal feed pressure that nozzle f_max values are referenced to.
  double referencePressure() const {return p_.regulated ? p_.p_reg_pa : p_.p0_pa;}

  double mass() const {return mass_;}
  double usedMass() const {return mass0_ - mass_;}

  // Remove gas; clamps at zero and returns the mass actually drawn.
  double draw(double dm_kg)
  {
    const double d = std::clamp(dm_kg, 0.0, mass_);
    mass_ -= d;
    return d;
  }

  void recharge()
  {
    mass0_ = massAt(p_.p0_pa);
    mass_ = mass0_;
  }

  const TankParams & params() const {return p_;}

private:
  TankParams p_;
  double mass0_ = 0.0;
  double mass_ = 0.0;
};

}  // namespace roadster_gz_plugins
