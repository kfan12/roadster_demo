// ins_ekf.hpp - IMU strapdown navigation + error-state EKF (plan section 5.10).
//
// Nominal state (integrated at the IMU rate): position p and velocity v of the
// IMU point in the world, attitude q (body -> world), accelerometer bias b_a,
// gyro bias b_g. The 15-dim error state [dp dv dth db_a db_g] uses a RIGHT
// (body-frame) attitude perturbation q = q_hat * Exp(dth), so the attitude
// error stays well defined through +-90 deg pitch and the 180 deg roll.
//
//   p' = v,  v' = R (f - b_a) + g,  q' = q * Exp((w - b_g) dt)
//
// Measurement updates (all exact-linearized, Joseph form):
//   bodyVelocity  v_body = R^T v                   (wheel speed + no side-slip / no hop)
//   zeroVelocity  v = 0                            (standing still)
//   gravity       f = R^T (0, 0, g) + b_a          (static: levels attitude, accel bias)
//   zeroRate      w = b_g                          (static: all three gyro biases)
//   surface       n . (p + R lever - s0) = 0       (a body point on a known surface)
// Pure math (Eigen only), unit-tested in test/test_ins_ekf.cpp.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace roadster_control
{

struct InsNoise
{
  double accel = 0.08;            // accelerometer white noise, per sample [m/s^2]
  double gyro = 0.009;            // gyro white noise, per sample [rad/s]
  double accel_bias_walk = 1e-4;  // [m/s^2 / sqrt(s)]
  double gyro_bias_walk = 1e-5;   // [rad/s / sqrt(s)]
};

class InsEkf
{
public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;
  using Quat = Eigen::Quaterniond;
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  static constexpr double kG = 9.81;

  explicit InsEkf(const InsNoise & n = InsNoise{})
  : n_(n) {}

  void reset(const Vec3 & p, const Quat & q, const Vec3 & v,
    double sigma_p, double sigma_v, double sigma_th, double sigma_ba, double sigma_bg)
  {
    p_ = p;
    v_ = v;
    q_ = q.normalized();
    ba_.setZero();
    bg_.setZero();
    P_.setZero();
    P_.block<3, 3>(0, 0) = Mat3::Identity() * sigma_p * sigma_p;
    P_.block<3, 3>(3, 3) = Mat3::Identity() * sigma_v * sigma_v;
    P_.block<3, 3>(6, 6) = Mat3::Identity() * sigma_th * sigma_th;
    P_.block<3, 3>(9, 9) = Mat3::Identity() * sigma_ba * sigma_ba;
    P_.block<3, 3>(12, 12) = Mat3::Identity() * sigma_bg * sigma_bg;
  }

  static Mat3 skew(const Vec3 & a)
  {
    Mat3 s;
    s << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
    return s;
  }

  static Quat expq(const Vec3 & th)
  {
    const double a = th.norm();
    if (a < 1e-12) {
      return Quat(1, 0.5 * th.x(), 0.5 * th.y(), 0.5 * th.z()).normalized();
    }
    return Quat(Eigen::AngleAxisd(a, th / a));
  }

  // f: specific force [m/s^2], w: angular rate [rad/s], both body frame
  void propagate(const Vec3 & f, const Vec3 & w, double dt)
  {
    const Mat3 R = q_.toRotationMatrix();
    const Vec3 fc = f - ba_;
    const Vec3 wc = w - bg_;
    const Vec3 a = R * fc + Vec3(0, 0, -kG);
    p_ += v_ * dt + 0.5 * a * dt * dt;
    v_ += a * dt;
    q_ = (q_ * expq(wc * dt)).normalized();

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * skew(fc) * dt;
    F.block<3, 3>(3, 9) = -R * dt;
    F.block<3, 3>(6, 6) = expq(-wc * dt).toRotationMatrix();
    F.block<3, 3>(6, 12) = -Mat3::Identity() * dt;
    Mat15 Q = Mat15::Zero();
    Q.block<3, 3>(3, 3) = Mat3::Identity() * std::pow(n_.accel * dt, 2);
    Q.block<3, 3>(6, 6) = Mat3::Identity() * std::pow(n_.gyro * dt, 2);
    Q.block<3, 3>(9, 9) = Mat3::Identity() * n_.accel_bias_walk * n_.accel_bias_walk * dt;
    Q.block<3, 3>(12, 12) = Mat3::Identity() * n_.gyro_bias_walk * n_.gyro_bias_walk * dt;
    P_ = F * P_ * F.transpose() + Q;
  }

  // body-frame velocity measurement with per-axis std dev
  // force: skip the innovation gate (fault recovery after persistent rejection)
  bool updateBodyVelocity(const Vec3 & z, const Vec3 & sigma, bool force = false)
  {
    const Mat3 Rt = q_.toRotationMatrix().transpose();
    const Vec3 vb = Rt * v_;
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Rt;
    H.block<3, 3>(0, 6) = skew(vb);
    return update<3>(z - vb, H, sigma.cwiseProduct(sigma).asDiagonal(), gate_ && !force);
  }

  bool updateZeroVelocity(double sigma)
  {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Mat3::Identity();
    return update<3>(-v_, H, Mat3::Identity() * sigma * sigma);
  }

  // standing still: the measured rate is pure gyro bias (zero-rate update).
  // Without it the yaw-rate bias is unobservable when static (gravity says
  // nothing about yaw) and wandered to 0.03 rad/s in test_ins_ekf.
  bool updateZeroRate(const Vec3 & w, double sigma)
  {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 12) = Mat3::Identity();
    return update<3>(w - bg_, H, Mat3::Identity() * sigma * sigma);
  }

  // static specific force = gravity reaction in the body frame (+ accel bias)
  bool updateGravity(const Vec3 & f, double sigma)
  {
    const Mat3 Rt = q_.toRotationMatrix().transpose();
    const Vec3 hb = Rt * Vec3(0, 0, kG);
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 6) = skew(hb);
    H.block<3, 3>(0, 9) = Mat3::Identity();
    return update<3>(f - (hb + ba_), H, Mat3::Identity() * sigma * sigma);
  }

  // a body point (p + R lever) lies on a surface: dist is its current signed
  // distance along the surface's unit normal n (world frame)
  bool updateSurface(double dist, const Vec3 & n, const Vec3 & lever, double sigma)
  {
    const Mat3 R = q_.toRotationMatrix();
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H.block<1, 3>(0, 0) = n.transpose();
    H.block<1, 3>(0, 6) = -n.transpose() * R * skew(lever);
    return update<1>(Eigen::Matrix<double, 1, 1>(-dist), H,
      Eigen::Matrix<double, 1, 1>(sigma * sigma));
  }

  const Vec3 & p() const {return p_;}
  const Vec3 & v() const {return v_;}
  const Quat & q() const {return q_;}
  const Vec3 & ba() const {return ba_;}
  const Vec3 & bg() const {return bg_;}
  const Mat15 & P() const {return P_;}

private:
  // Innovation gate (wheel velocity only - skidding or spinning wheels are the
  // outliers here; gating the geometric surface constraint locked the filter
  // out after the spawn settle and let height drift 5.6 m): a measurement whose
  // normalized innovation r' S^-1 r
  // exceeds the chi-square bound is rejected (skidding wheels, a missed
  // contact, an impact). Returns whether it was applied.
  template<int M>
  bool update(
    const Eigen::Matrix<double, M, 1> & r, const Eigen::Matrix<double, M, 15> & H,
    const Eigen::Matrix<double, M, M> & Rn, bool gate = false)
  {
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + Rn;
    const Eigen::Matrix<double, M, M> Si = S.inverse();
    const double d2 = r.dot(Si * r);
    // 99.9% chi-square bounds for 1 and 3 degrees of freedom
    if (gate && d2 > (M == 1 ? 10.83 : 16.27)) {
      ++rejected_;
      return false;
    }
    const Eigen::Matrix<double, 15, M> K = P_ * H.transpose() * Si;
    const Eigen::Matrix<double, 15, 1> dx = K * r;
    const Mat15 IKH = Mat15::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * Rn * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    p_ += dx.segment<3>(0);
    v_ += dx.segment<3>(3);
    q_ = (q_ * expq(dx.segment<3>(6))).normalized();
    ba_ += dx.segment<3>(9);
    bg_ += dx.segment<3>(12);
    return true;
  }

  InsNoise n_;
  bool gate_ = true;
  unsigned long rejected_ = 0;

public:
  void setGating(bool on) {gate_ = on;}
  unsigned long rejected() const {return rejected_;}

private:
  Vec3 p_ = Vec3::Zero(), v_ = Vec3::Zero(), ba_ = Vec3::Zero(), bg_ = Vec3::Zero();
  Quat q_ = Quat::Identity();
  Mat15 P_ = Mat15::Identity();
};

}  // namespace roadster_control
