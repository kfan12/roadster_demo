// attitude.hpp - quaternion attitude control + flip reference (plan 5.6).
//
// Everything is quaternion / rotation-vector math: the stunt passes through
// +-90 deg of pitch on the ramp and 180 deg of roll in the flip, where any
// Euler-angle formulation hits gimbal lock.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace roadster_control
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

// Minimum-jerk blend s(tau) from 0 to 1 over tau in [0, 1], with zero velocity
// and acceleration at both ends; returns {s, ds/dtau, d2s/dtau2}.
struct Blend
{
  double s, ds, dds;
};

inline Blend minJerk(double tau)
{
  tau = std::clamp(tau, 0.0, 1.0);
  const double t2 = tau * tau, t3 = t2 * tau;
  return {t3 * (10 - 15 * tau + 6 * t2),
    30 * t2 * (1 - 2 * tau + t2),
    60 * tau * (1 - 3 * tau + 2 * t2)};
}

// Rotation vector (axis * angle, angle in [0, pi]) of q_ref^-1 * q: the
// attitude error expressed in the BODY frame of q.
inline Vec3 attitudeError(const Quat & q_ref, const Quat & q)
{
  Quat qe = q_ref.conjugate() * q;
  if (qe.w() < 0) {
    qe.coeffs() *= -1;  // shortest way round
  }
  const double vn = qe.vec().norm();
  if (vn < 1e-12) {
    return Vec3::Zero();
  }
  const double angle = 2.0 * std::atan2(vn, qe.w());
  return qe.vec() / vn * angle;
}

// A rotation of `total_angle` about one body axis of the start attitude q0,
// shaped by a minimum-jerk profile over `duration` starting at t_start, then
// an optional small correction `c` (a rotation vector in the body frame the
// flip ends in) on its own minimum-jerk profile over `correction_duration`:
//   q(t) = q0 A(a(t)) C(s(t)),  A = rotation a about the axis, C(s) = exp(s c)
// ending at q0 A(total) exp(c). It levels a flip that starts pitched (off the
// tilted ceiling), which the roll alone would end nose-up. The correction
// comes after the roll, not blended into it: during a fast roll a fixed
// pitch correction's axis sweeps round the body, and mid-roll it is a body
// YAW rate, which only the rear nozzles can torque (with a big forward push:
// 15 kN.m of yaw, 16 kN.s of rear thrust and a 16 m/s touchdown, tilt_imu1).
// One at a time, the body rate and acceleration are those of the moving part.
class FlipReference
{
public:
  FlipReference(const Quat & q0, const Vec3 & body_axis, double total_angle,
    double t_start, double duration, const Vec3 & correction = Vec3::Zero(),
    double correction_duration = 0.0)
  : q0_(q0), axis_(body_axis.normalized()), total_(total_angle), t0_(t_start), T_(duration),
    c_(correction), Tc_(correction.norm() > 1e-12 ? correction_duration : 0.0) {}

  // The correction that makes a flip of `total_angle` about `body_axis` from
  // q0 end upright (body z along world z) with its heading kept: the end
  // attitude's forward axis, made horizontal. Undefined (zero) if that axis
  // ends near vertical.
  static Vec3 levelingCorrection(const Quat & q0, const Vec3 & body_axis, double total_angle)
  {
    const Mat3 R_end = (q0 * Quat(Eigen::AngleAxisd(total_angle, body_axis.normalized())))
      .toRotationMatrix();
    Vec3 x = R_end.col(0);
    x.z() = 0.0;
    if (x.norm() < 0.1) {
      return Vec3::Zero();
    }
    x.normalize();
    Mat3 R_level;
    R_level.col(0) = x;
    R_level.col(2) = Vec3::UnitZ();
    R_level.col(1) = Vec3::UnitZ().cross(x);
    const Eigen::AngleAxisd c(R_end.transpose() * R_level);
    return c.axis() * c.angle();
  }

  struct Sample
  {
    Quat q;        // reference attitude
    Vec3 w;        // reference body rate
    Vec3 alpha;    // reference body angular acceleration
    double angle;  // rotation so far about the axis
  };

  Sample at(double t) const
  {
    const Blend b = minJerk(T_ > 0 ? (t - t0_) / T_ : 1.0);
    const bool moving = T_ > 0 && t > t0_ && t < t0_ + T_;
    const double a = total_ * b.s;
    const double t1 = t0_ + T_;  // the correction's start
    const Blend bc = minJerk(Tc_ > 0 ? (t - t1) / Tc_ : 1.0);
    const bool correcting = Tc_ > 0 && t > t1 && t < t1 + Tc_;
    const double cn = c_.norm();
    const Quat C = cn > 1e-12 ? Quat(Eigen::AngleAxisd(bc.s * cn, c_ / cn)) : Quat::Identity();
    Sample s;
    s.q = q0_ * Quat(Eigen::AngleAxisd(a, axis_)) * C;
    if (moving) {
      s.w = axis_ * total_ * b.ds / T_;
      s.alpha = axis_ * total_ * b.dds / (T_ * T_);
    } else if (correcting) {
      s.w = c_ * bc.ds / Tc_;
      s.alpha = c_ * bc.dds / (Tc_ * Tc_);
    } else {
      s.w = Vec3::Zero();
      s.alpha = Vec3::Zero();
    }
    s.angle = a;
    return s;
  }

  // the roll's end, and the end of the whole reference (with the correction)
  double rollEndTime() const {return t0_ + T_;}
  double endTime() const {return t0_ + T_ + Tc_;}
  const Vec3 & axis() const {return axis_;}

private:
  Quat q0_;
  Vec3 axis_;
  double total_, t0_, T_;
  Vec3 c_;
  double Tc_;
};

// Body torque for a PD on the rotation-vector error with reference rate and
// acceleration feed-forward, plus gyroscopic compensation:
//   tau = I (alpha_ref - Kp e - Kd (w - w_ref)) + w x (I w)
struct AttitudeGains
{
  Vec3 kp{36, 36, 36};  // [1/s^2]
  Vec3 kd{10.8, 10.8, 10.8};  // [1/s]
};

inline Vec3 attitudeTorque(
  const Mat3 & inertia, const AttitudeGains & g, const Quat & q, const Vec3 & w_body,
  const Quat & q_ref, const Vec3 & w_ref, const Vec3 & alpha_ref)
{
  const Vec3 e = attitudeError(q_ref, q);
  const Vec3 acc = alpha_ref - g.kp.cwiseProduct(e) - g.kd.cwiseProduct(w_body - w_ref);
  return inertia * acc + w_body.cross(inertia * w_body);
}

}  // namespace roadster_control
