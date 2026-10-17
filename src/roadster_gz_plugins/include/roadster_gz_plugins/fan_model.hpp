// fan_model.hpp - suction fan downforce + analytic surface gap (plan 5.4).
//
// F = f_max * w^2 * g(h), along the car's body -z (into whatever surface the
// wheels are on). w is the normalized fan speed, a first-order lag of the
// command. g(h) is the ground-effect falloff: 1 up to h_seal, linear to 0 at
// h_max. Fortress exposes no ray queries to systems, so h comes from the
// world's analytic surfaces (maps/<world>.json "surfaces", written by the
// world generator): planes with a validity box, and concave cylindrical arcs
// about an axis parallel to world y (the inverted ramp).
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace roadster_gz_plugins
{

struct Vec3
{
  double x = 0, y = 0, z = 0;
};

inline Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline double dot(const Vec3 & a, const Vec3 & b) {return a.x * b.x + a.y * b.y + a.z * b.z;}

struct PlaneSurface
{
  Vec3 point;
  Vec3 normal;            // unit, pointing to the free (drivable) side
  Vec3 lo, hi;            // the query point must lie inside this box
};

// Inner (concave) side of a cylinder with axis parallel to world y through
// (cx, cz). Angles are measured in the x-z plane as atan2(z - cz, x - cx),
// and the arc covers [theta0, theta1] (radians, theta0 < theta1).
struct ArcSurface
{
  double cx = 0, cz = 0, radius = 0;
  double theta0 = 0, theta1 = 0;
  double y_min = 0, y_max = 0;
};

struct GapResult
{
  double h = std::numeric_limits<double>::infinity();
  Vec3 normal;  // free-side normal of the surface the gap is measured to
};

class SurfaceSet
{
public:
  std::vector<PlaneSurface> planes;
  std::vector<ArcSurface> arcs;

  // Gap from point p to the nearest surface whose free side faces the car's
  // underside: its normal must be within max_angle of the car's body +z.
  GapResult gap(const Vec3 & p, const Vec3 & body_z, double max_angle_rad) const
  {
    const double cos_max = std::cos(max_angle_rad);
    GapResult best;
    for (const auto & s : planes) {
      if (p.x < s.lo.x || p.x > s.hi.x || p.y < s.lo.y || p.y > s.hi.y ||
        p.z < s.lo.z || p.z > s.hi.z)
      {
        continue;
      }
      if (dot(s.normal, body_z) < cos_max) {
        continue;
      }
      const double h = dot(p - s.point, s.normal);
      if (h > -0.5 && h < best.h) {  // slightly negative = penetration
        best.h = h;
        best.normal = s.normal;
      }
    }
    for (const auto & a : arcs) {
      if (p.y < a.y_min || p.y > a.y_max) {
        continue;
      }
      const double dx = p.x - a.cx, dz = p.z - a.cz;
      const double r = std::hypot(dx, dz);
      if (r < 1e-6) {
        continue;
      }
      const double th = std::atan2(dz, dx);
      if (!angleInRange(th, a.theta0, a.theta1)) {
        continue;
      }
      const Vec3 n{-dx / r, 0.0, -dz / r};  // inward = free side of a concave arc
      if (dot(n, body_z) < cos_max) {
        continue;
      }
      const double h = a.radius - r;
      if (h > -0.5 && h < best.h) {
        best.h = h;
        best.normal = n;
      }
    }
    return best;
  }

  static bool angleInRange(double th, double t0, double t1)
  {
    // shift th by multiples of 2 pi into [t0, t0 + 2 pi)
    const double two_pi = 2.0 * M_PI;
    double d = std::fmod(th - t0, two_pi);
    if (d < 0) {
      d += two_pi;
    }
    return d <= (t1 - t0);
  }
};

struct FanParams
{
  double f_max_n = 0.0;
  double tau_rise_s = 0.3;  // spin-up lag
  double tau_fall_s = 0.3;  // spin-down lag; 0 = suction gone at once on cut-off
  double h_seal_m = 0.35;
  double h_max_m = 1.0;
};

class FanModel
{
public:
  explicit FanModel(const FanParams & p, double w0 = 0.0)
  : p_(p), w_(std::clamp(w0, 0.0, 1.0)) {}

  double groundEffect(double h) const
  {
    if (h <= p_.h_seal_m) {
      return 1.0;
    }
    if (h >= p_.h_max_m) {
      return 0.0;
    }
    return (p_.h_max_m - h) / (p_.h_max_m - p_.h_seal_m);
  }

  // Advance the spin state by dt and return the suction force magnitude [N].
  double step(double cmd, double h, double dt)
  {
    cmd = std::clamp(cmd, 0.0, 1.0);
    const double tau = cmd >= w_ ? p_.tau_rise_s : p_.tau_fall_s;
    w_ += (cmd - w_) * (tau > 0 ? 1.0 - std::exp(-dt / tau) : 1.0);
    force_ = p_.f_max_n * w_ * w_ * groundEffect(h);
    return force_;
  }

  double speed() const {return w_;}
  double force() const {return force_;}

private:
  FanParams p_;
  double w_ = 0.0;
  double force_ = 0.0;
};

}  // namespace roadster_gz_plugins
