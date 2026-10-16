// Unit tests for the IMU strapdown + error-state EKF.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "roadster_control/ins_ekf.hpp"

using roadster_control::InsEkf;
using Vec3 = InsEkf::Vec3;
using Quat = InsEkf::Quat;

TEST(InsEkf, StaticNoiseFreeStaysPut)
{
  InsEkf f;
  f.reset(Vec3(1, 2, 3), Quat::Identity(), Vec3::Zero(), 0.01, 0.01, 0.01, 0.01, 0.001);
  for (int i = 0; i < 2000; ++i) {
    f.propagate(Vec3(0, 0, InsEkf::kG), Vec3::Zero(), 0.005);
  }
  EXPECT_LT((f.p() - Vec3(1, 2, 3)).norm(), 1e-9);
  EXPECT_LT(f.v().norm(), 1e-9);
}

TEST(InsEkf, ConstantRateRotationIntegrates)
{
  InsEkf f;
  f.reset(Vec3::Zero(), Quat::Identity(), Vec3::Zero(), 0.01, 0.01, 0.01, 0.01, 0.001);
  // half a turn about body x in 1 s, free fall (zero specific force)
  for (int i = 0; i < 200; ++i) {
    f.propagate(Vec3::Zero(), Vec3(M_PI, 0, 0), 0.005);
  }
  const Quat want(Eigen::AngleAxisd(M_PI, Vec3::UnitX()));
  EXPECT_NEAR(std::abs(f.q().dot(want)), 1.0, 1e-9);
  EXPECT_NEAR(f.v().z(), -InsEkf::kG, 1e-9);
  EXPECT_NEAR(f.p().z(), -0.5 * InsEkf::kG, 1e-9);
}

TEST(InsEkf, StaticUpdatesLevelAttitudeAndFindGyroBias)
{
  // truth: level and still; the gyro has biases (0.02, 0, -0.01) rad/s; the
  // filter starts 3 deg off in roll. Static updates: zero velocity, gravity,
  // zero rate.
  std::mt19937 rng(1);
  std::normal_distribution<double> na(0, 0.08), ng(0, 0.009);
  InsEkf f;
  f.reset(Vec3::Zero(), Quat(Eigen::AngleAxisd(0.05, Vec3::UnitX())), Vec3::Zero(),
    0.01, 0.01, 0.1, 0.02, 0.05);
  for (int i = 0; i < 4000; ++i) {
    const Vec3 fm(na(rng), na(rng), InsEkf::kG + na(rng));
    const Vec3 wm(0.02 + ng(rng), ng(rng), -0.01 + ng(rng));
    f.propagate(fm, wm, 0.005);
    if (i % 4 == 0) {
      f.updateZeroVelocity(0.02);
      f.updateGravity(fm, 0.3);
      f.updateZeroRate(wm, 0.02);
    }
  }
  const Vec3 up = f.q() * Vec3::UnitZ();
  EXPECT_LT(std::acos(std::min(1.0, up.z())), 0.01);  // levelled to < 0.6 deg
  EXPECT_LT(Eigen::AngleAxisd(f.q()).angle(), 0.02);  // yaw held too (bias known)
  EXPECT_NEAR(f.bg().x(), 0.02, 0.002);
  EXPECT_NEAR(f.bg().z(), -0.01, 0.002);
  EXPECT_LT(f.v().norm(), 0.02);
}

TEST(InsEkf, WheelSpeedBoundsDriftWhileDriving)
{
  // truth: straight line along x at 6 m/s, level; accel bias 0.05 on x would
  // drift 0.05 * 10^2 / 2 = 2.5 m in 10 s without aiding
  InsEkf f;
  f.reset(Vec3::Zero(), Quat::Identity(), Vec3(6, 0, 0), 0.01, 0.05, 0.01, 0.1, 0.001);
  for (int i = 0; i < 2000; ++i) {
    f.propagate(Vec3(0.05, 0, InsEkf::kG), Vec3::Zero(), 0.005);
    if (i % 4 == 0) {
      f.updateBodyVelocity(Vec3(6, 0, 0), Vec3(0.1, 0.05, 0.05));
    }
  }
  EXPECT_NEAR(f.p().x(), 60.0, 0.3);
  EXPECT_NEAR(f.v().x(), 6.0, 0.05);
  EXPECT_NEAR(f.ba().x(), 0.05, 0.02);
}

// Noise-free car driving round the inside of an R = 8 m loop (the Demo A ramp)
// at constant speed, with the exact IMU, body-velocity and per-wheel surface
// measurements that geometry implies. A consistent filter must not drift.
TEST(InsEkf, NoiseFreeLoopDoesNotDrift)
{
  const double Rr = 8.0, r = 0.33, a = 1.35, half_track = 0.8;
  const double rho = std::sqrt((Rr - r) * (Rr - r) - a * a);  // IMU radius from the arc centre
  const double v_c = 6.0, om = v_c / (Rr - r);               // wheel-centre speed, pitch rate
  const Vec3 c(0, 0, Rr);
  auto pose = [&](double al, Vec3 & p, Quat & q) {
      q = Quat(Eigen::AngleAxisd(-al, Vec3::UnitY()));
      p = c + rho * Vec3(std::sin(al), 0, -std::cos(al));
    };
  Vec3 p0;
  Quat q0;
  pose(0.0, p0, q0);
  InsEkf f;
  f.reset(p0, q0, Vec3(rho * om, 0, 0), 0.02, 0.02, 0.01, 0.02, 0.01);
  const double dt = 0.005;
  double max_att = 0.0, max_pos = 0.0;
  for (int i = 1; i * dt * om < M_PI * 0.95; ++i) {
    const double al = i * dt * om;
    Vec3 p;
    Quat q;
    pose(al, p, q);
    const Vec3 acc = -rho * om * om * Vec3(std::sin(al), 0, -std::cos(al));
    const Vec3 fb = q.conjugate() * (acc - Vec3(0, 0, -InsEkf::kG));
    f.propagate(fb, Vec3(0, -om, 0), dt);
    if (i % 4 == 0) {
      f.updateBodyVelocity(Vec3(rho * om, 0, 0), Vec3(0.15, 0.08, 0.08));
      for (double sx : {a, -a}) {
        for (double sy : {half_track, -half_track}) {
          const Vec3 lever(sx, sy, 0);
          const Vec3 pc = f.p() + f.q() * lever;
          const Vec3 d = pc - c;
          const double rr = std::hypot(d.x(), d.z());
          const double dist = (Rr - rr) - r;               // centre one radius from the surface
          const Vec3 n(-d.x() / rr, 0, -d.z() / rr);         // inward = free side
          f.updateSurface(dist, n, lever, 0.02);
        }
      }
    }
    max_att = std::max(max_att, Eigen::AngleAxisd(q.conjugate() * f.q()).angle());
    max_pos = std::max(max_pos, (f.p() - p).norm());
  }
  EXPECT_LT(max_att, 0.005);  // < 0.3 deg over 170 deg of loop
  EXPECT_LT(max_pos, 0.05);  // first-order integration of curved motion: ~3 cm
}

TEST(InsEkf, SurfaceUpdatePinsHeight)
{
  // a point 0.33 m below the IMU must be on the z = 16 plane; estimate starts 0.3 m off
  InsEkf f;
  f.reset(Vec3(0, 0, 16.6), Quat::Identity(), Vec3::Zero(), 0.5, 0.01, 0.001, 0.01, 0.001);
  const Vec3 lever(0, 0, -0.33);
  for (int i = 0; i < 20; ++i) {
    const double dist = (f.p() + f.q() * lever).z() - 16.0;
    f.updateSurface(dist, Vec3(0, 0, 1), lever, 0.02);
  }
  EXPECT_NEAR(f.p().z(), 16.33, 0.01);
}
