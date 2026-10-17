// Unit tests: attitude math, flip reference, landing profile, allocator.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "roadster_control/allocator.hpp"
#include "roadster_control/attitude.hpp"
#include "roadster_control/landing.hpp"

using namespace roadster_control;

TEST(MinJerk, EndpointsAndSymmetry)
{
  EXPECT_DOUBLE_EQ(minJerk(0).s, 0.0);
  EXPECT_DOUBLE_EQ(minJerk(1).s, 1.0);
  EXPECT_NEAR(minJerk(0.5).s, 0.5, 1e-12);
  EXPECT_NEAR(minJerk(0).ds, 0.0, 1e-12);
  EXPECT_NEAR(minJerk(1).ds, 0.0, 1e-12);
  EXPECT_NEAR(minJerk(0.5).ds, 1.875, 1e-12);  // peak rate of the 5th-order blend
  EXPECT_NEAR(minJerk(0.5).dds, 0.0, 1e-12);
}

TEST(Attitude, ErrorIsBodyFrameRotationVector)
{
  const Quat q_ref = Quat::Identity();
  const Quat q(Eigen::AngleAxisd(0.3, Vec3::UnitX()));
  const Vec3 e = attitudeError(q_ref, q);
  EXPECT_NEAR(e.x(), 0.3, 1e-12);
  EXPECT_NEAR(e.tail<2>().norm(), 0.0, 1e-12);
  // shortest way round: 350 deg about z is -10 deg
  const Quat q2(Eigen::AngleAxisd(2 * M_PI - 0.1745, Vec3::UnitZ()));
  EXPECT_NEAR(attitudeError(q_ref, q2).z(), -0.1745, 1e-9);
  // no gimbal lock at 90 deg pitch: a small roll on top is still small
  const Quat pitched(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()));
  const Quat pr = pitched * Quat(Eigen::AngleAxisd(0.05, Vec3::UnitX()));
  EXPECT_NEAR(attitudeError(pitched, pr).x(), 0.05, 1e-9);
}

TEST(Attitude, PdTorqueSignsAndGyroTerm)
{
  const Mat3 I = Vec3(584.5, 3119.5, 3562.0).asDiagonal();
  AttitudeGains g;
  const Quat q(Eigen::AngleAxisd(0.1, Vec3::UnitX()));
  const Vec3 tau = attitudeTorque(I, g, q, Vec3::Zero(), Quat::Identity(), Vec3::Zero(),
      Vec3::Zero());
  EXPECT_NEAR(tau.x(), -584.5 * 36 * 0.1, 1e-6);  // pushes back toward the reference
  const Vec3 w(1.0, 0.5, 0.0);
  const Vec3 tg = attitudeTorque(I, g, Quat::Identity(), w, Quat::Identity(), w, Vec3::Zero());
  EXPECT_NEAR((tg - w.cross(I * w)).norm(), 0.0, 1e-9);  // on-reference: gyroscopic only
}

TEST(Flip, HalfTurnFromInvertedEndsUpright)
{
  const Quat inverted(Eigen::AngleAxisd(M_PI, Vec3::UnitX()));
  FlipReference f(inverted, Vec3::UnitX(), M_PI, 1.0, 0.8);
  EXPECT_TRUE(f.at(0.5).q.isApprox(inverted));
  const auto end = f.at(1.8);
  EXPECT_NEAR(std::abs(end.q.w()), 1.0, 1e-9);  // identity up to sign: upright
  EXPECT_NEAR(end.angle, M_PI, 1e-12);
  const auto mid = f.at(1.4);
  EXPECT_NEAR(mid.w.x(), M_PI * 1.875 / 0.8, 1e-9);
  EXPECT_NEAR(mid.alpha.norm(), 0.0, 1e-9);
}

TEST(Landing, FreeFallThenLatchedBrake)
{
  LandingBrake b{LandingParams{}};
  // slow descent high up: above the profile, no thrust
  EXPECT_DOUBLE_EQ(b.upForce(10.0, -3.0, 1800, 1e6), 0.0);
  EXPECT_FALSE(b.active());
  // descending exactly at the profile speed: feed-forward m (g + a_b)
  const double v = b.profileSpeed(5.0);
  EXPECT_NEAR(b.upForce(5.0, -v, 1800, 1e6), 1800 * (9.81 + 14.7), 1e-6);
  EXPECT_TRUE(b.active());
  // latched: even if it slows below the profile it keeps braking (clamped >= 0)
  EXPECT_GE(b.upForce(5.0, -0.5 * v, 1800, 1e6), 0.0);
  EXPECT_TRUE(b.active());
  EXPECT_NEAR(b.profileSpeed(0.0), 1.5, 1e-12);
  EXPECT_DOUBLE_EQ(b.upForce(1.0, -30.0, 1800, 5000.0), 5000.0);  // clamped to capacity
}

TEST(Landing, FlareBrakesHarderNearTheGround)
{
  LandingParams p;
  p.touchdown_speed = 0.5;
  p.flare_height = 1.0;
  p.flare_accel = 24.5;
  LandingBrake b{p};
  // continuous at h_f: the flare stage alone takes 0.5 m/s up to h_f
  const double v_f = std::sqrt(0.25 + 2 * 24.5 * 1.0);
  EXPECT_NEAR(b.profileSpeed(1.0), v_f, 1e-12);
  EXPECT_NEAR(b.profileSpeed(5.0), std::sqrt(v_f * v_f + 2 * 14.7 * 4.0), 1e-12);
  EXPECT_NEAR(b.profileSpeed(0.0), 0.5, 1e-12);
  EXPECT_NEAR(b.profileSpeed(-0.1), 0.5, 1e-12);
  // on the profile: feed-forward a_b above the flare, a_f below it
  EXPECT_NEAR(b.upForce(5.0, -b.profileSpeed(5.0), 1800, 1e6), 1800 * (9.81 + 14.7), 1e-6);
  EXPECT_NEAR(b.upForce(0.5, -b.profileSpeed(0.5), 1800, 1e6), 1800 * (9.81 + 24.5), 1e-6);
}

TEST(Landing, FlareStartsEarlyAndCorrectsHarder)
{
  LandingParams p;
  p.flare_height = 1.0;
  p.flare_accel = 19.6;
  p.flare_kv = 10.0;
  p.flare_lead_time = 0.04;
  LandingBrake b{p};
  // latches lead_time early: (g + a_b) lead_time below the profile speed
  const double v0 = b.profileSpeed(5.0) - (9.81 + 14.7) * 0.04;
  EXPECT_DOUBLE_EQ(b.upForce(5.0, -(v0 - 0.01), 1800, 1e6), 0.0);
  EXPECT_FALSE(b.active());
  EXPECT_GT(b.upForce(5.0, -(v0 + 0.01), 1800, 1e6), 0.0);
  EXPECT_TRUE(b.active());
  // 1.2 m up at 6 m/s: 0.24 m of lead puts it in the flare already
  const double v = b.profileSpeed(1.2);
  EXPECT_NEAR(b.upForce(1.2, -v, 1800, 1e6), 1800 * (9.81 + 19.6), 1e-6);
  // 0.5 m/s too fast in the flare: k_vf, not k_v, on the error
  const double v2 = b.profileSpeed(0.5) + 0.5;
  EXPECT_NEAR(b.upForce(0.5, -v2, 1800, 1e6), 1800 * (9.81 + 19.6 + 10.0 * 0.5), 1e-6);
}

TEST(Landing, ReachesTouchdownSpeedAtTouchdownHeightAndHoldsIt)
{
  LandingParams p;
  p.touchdown_speed = 0.5;
  p.touchdown_height = 0.05;
  p.flare_height = 1.0;
  p.flare_accel = 19.6;
  p.flare_kv = 10.0;
  LandingBrake b{p};
  EXPECT_NEAR(b.profileSpeed(0.05), 0.5, 1e-12);
  EXPECT_NEAR(b.profileSpeed(0.02), 0.5, 1e-12);
  EXPECT_NEAR(b.profileSpeed(1.05), std::sqrt(0.25 + 2 * 19.6), 1e-12);
  EXPECT_GT(b.upForce(2.0, -30.0, 1800, 1e6), 0.0);  // latch
  // below h_td on the profile: just hold the weight, no deceleration
  EXPECT_NEAR(b.upForce(0.03, -0.5, 1800, 1e6), 1800 * 9.81, 1e-6);
}

TEST(Landing, FlareEndsEarlyByTheThrustFallLag)
{
  LandingParams p;
  p.touchdown_speed = 0.5;
  p.flare_height = 1.0;
  p.flare_accel = 19.6;
  p.flare_kv = 10.0;
  p.thrust_fall_time = 0.02;
  LandingBrake b{p};
  EXPECT_GT(b.upForce(2.0, -30.0, 1800, 1e6), 0.0);  // latch
  // on the profile at 0.7 m/s: above v_td / 2 + a_f t_fall = 0.642, still braking
  const double h1 = (0.7 * 0.7 - 0.25) / (2 * 19.6);
  EXPECT_NEAR(b.upForce(h1, -0.7, 1800, 1e6), 1800 * (9.81 + 19.6), 1e-6);
  // at 0.6 m/s: the decaying thrust will take off the rest, feed-forward ends
  const double h2 = (0.6 * 0.6 - 0.25) / (2 * 19.6);
  EXPECT_NEAR(b.upForce(h2, -0.6, 1800, 1e6), 1800 * 9.81, 1e-6);
}

namespace
{
std::vector<NozzleGeom> roadsterNozzles(double mg)
{
  // same layout as roadster_description/config/nozzles.yaml (COM at origin)
  struct N {const char * n; double x, y, z, dx, dy, dz, k;};
  const N tab[] = {
    {"down_front_left", 1.9, 0.75, 0, 0, 0, 1, 1.1}, {"down_front_right", 1.9, -0.75, 0, 0, 0, 1, 1.1},
    {"down_rear_left", -1.9, 0.75, 0, 0, 0, 1, 1.1}, {"down_rear_right", -1.9, -0.75, 0, 0, 0, 1, 1.1},
    {"rear_main_left_inner", -2.25, 0.2, 0.3, 1, 0, 0, 1.125},
    {"rear_main_left_outer", -2.25, 0.55, 0.3, 1, 0, 0, 1.125},
    {"rear_main_right_inner", -2.25, -0.2, 0.3, 1, 0, 0, 1.125},
    {"rear_main_right_outer", -2.25, -0.55, 0.3, 1, 0, 0, 1.125},
    {"side_left", 0, 0.9, 0, 0, -1, 0, 0.5},
    {"side_right", 0, -0.9, 0, 0, 1, 0, 0.5}};
  std::vector<NozzleGeom> v;
  for (const auto & t : tab) {
    v.push_back({t.n, {t.x, t.y, t.z}, {t.dx, t.dy, t.dz}, t.k * mg});
  }
  return v;
}
}  // namespace

TEST(Allocator, MeetsAFeasibleWrench)
{
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  Vec6 w;
  w << 0, 0, 2.0 * mg, 3000, -2000, 0;  // lift 2 m g, roll + pitch torque
  Vec6 wt;
  wt << 1, 1, 1, 1, 1, 1;
  const auto r = a.solve(w, wt, 1.0, 1e-8);
  EXPECT_TRUE(r.qp_ok);
  EXPECT_LT((r.achieved - w).norm() / w.norm(), 1e-3);
  EXPECT_GE(r.u.minCoeff(), 0.0);
  EXPECT_LE(r.u.maxCoeff(), 1.0);
}

TEST(Allocator, SaturatesWithinBoundsAndScalesWithFeed)
{
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  Vec6 w;
  w << 0, 0, 10.0 * mg, 0, 0, 0;  // more lift than the 4.4 m g the down nozzles have
  Vec6 wt;
  wt << 1, 1, 1, 1, 1, 1;
  const auto full = a.solve(w, wt, 1.0, 1e-8);
  EXPECT_NEAR(full.achieved(2), 4.4 * mg, 1e-2 * mg);
  EXPECT_LE(full.u.maxCoeff(), 1.0 + 1e-9);
  const auto half = a.solve(w, wt, 0.5, 1e-8);
  EXPECT_NEAR(half.achieved(2), 2.2 * mg, 1e-2 * mg);
  const auto none = a.solve(w, wt, 0.0, 1e-8);
  EXPECT_DOUBLE_EQ(none.u.norm(), 0.0);
}

TEST(Allocator, RollsOnOneSidesBottomPair)
{
  // no roof nozzles: a +roll torque can only come from the LEFT bottom pair
  // (+z at y > 0), and costs a body +z force of tau / 0.75 m
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  Vec6 w;
  w << 0, 0, 0, 5000, 0, 0;
  Vec6 wt;
  wt << 0.01, 0.3, 0.01, 1, 1, 1;  // the node's flight weights
  const auto r = a.solve(w, wt, 1.0, 1e-4);
  // within 3%: the small Fz weight trades a little torque for less of the
  // unavoidable lift (the flip's attitude feedback makes up the difference)
  EXPECT_NEAR(r.achieved(3), 5000, 150);
  EXPECT_NEAR(r.achieved.segment<2>(4).norm(), 0, 50);
  const double left_down = r.u(0) + r.u(2), right_down = r.u(1) + r.u(3);
  EXPECT_GT(left_down, 10 * right_down - 1e-6);
  EXPECT_NEAR(r.achieved(2), 5000 / 0.75, 0.04 * 5000 / 0.75);
  EXPECT_LT(r.u.segment<4>(4).norm(), 1e-3);  // rear nozzles shut
}

TEST(Allocator, SideNozzleCancelsTheFlipsSidewaysPush)
{
  // early in the kick flip (rolled 160 deg from upright): the roll torque's
  // bottom-pair force has a world-y component, and with a lateral weight the
  // side nozzle facing the other way pushes it back to (almost) zero
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  const Eigen::Matrix3d R =
    Eigen::AngleAxisd(160.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Vec6 w;
  w << 0, 0, 0, 10000, 0, 0;
  Vec6 wt;
  wt << 0.01, 0.3, 0.01, 1, 1, 1;
  const auto with = a.solve(w, wt, 1.0, 1e-4, R);
  wt(1) = 0.0;
  const auto without = a.solve(w, wt, 1.0, 1e-4, R);
  const double fy_with = (R * with.achieved.head<3>()).y();
  const double fy_without = (R * without.achieved.head<3>()).y();
  EXPECT_NEAR(with.achieved(3), 10000, 300);  // the lateral weight costs no more torque than Fz's
  EXPECT_GT(std::abs(fy_without), 0.1 * mg);  // tau / 0.75 m x sin(20 deg) ~ 0.26 m g
  EXPECT_LT(std::abs(fy_with), 0.02 * mg);
  EXPECT_GT(with.u(8) + with.u(9), 0.2);      // a side nozzle carries it
}

TEST(Allocator, SideNozzlesBrakeLateralVelocityWhenUpright)
{
  // after the flip: a world -y demand (braking a +y drift) comes from the
  // side_left nozzle (force -y) alone, with no torque
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  Vec6 w;
  w << 0, -0.3 * mg, 0, 0, 0, 0;
  Vec6 wt;
  wt << 0.05, 0.5, 1, 1, 1, 1;  // the node's brake weights
  const auto r = a.solve(w, wt, 1.0, 1e-4);
  EXPECT_NEAR(r.achieved(1), -0.3 * mg, 0.01 * mg);
  EXPECT_LT(r.achieved.tail<3>().norm(), 10.0);
  EXPECT_NEAR(r.u(8), 0.6, 0.02);  // 0.3 / 0.5 m g
  EXPECT_LT(r.u(9), 1e-3);
}
