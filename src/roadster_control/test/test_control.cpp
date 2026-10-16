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

namespace
{
std::vector<NozzleGeom> roadsterNozzles(double mg)
{
  // same layout as roadster_description/config/nozzles.yaml (COM at origin)
  struct N {const char * n; double x, y, z, dx, dy, dz, k;};
  const N tab[] = {
    {"down_front_left", 1.9, 0.75, 0, 0, 0, 1, 1.1}, {"down_front_right", 1.9, -0.75, 0, 0, 0, 1, 1.1},
    {"down_rear_left", -1.9, 0.75, 0, 0, 0, 1, 1.1}, {"down_rear_right", -1.9, -0.75, 0, 0, 0, 1, 1.1},
    {"rear_main_left", -2.25, 0.25, 0.3, 1, 0, 0, 2.25},
    {"rear_main_right", -2.25, -0.25, 0.3, 1, 0, 0, 2.25},
    {"roof_front_left", 1.9, 0.75, 0.7, 0, 0, -1, 0.45},
    {"roof_front_right", 1.9, -0.75, 0.7, 0, 0, -1, 0.45},
    {"roof_rear_left", -1.9, 0.75, 0.7, 0, 0, -1, 0.45},
    {"roof_rear_right", -1.9, -0.75, 0.7, 0, 0, -1, 0.45}};
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

TEST(Allocator, PureRollTorqueIsAForceFreeCouple)
{
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  Vec6 w;
  w << 0, 0, 0, 5000, 0, 0;
  Vec6 wt;
  wt << 0.01, 0.01, 0.01, 1, 1, 1;  // flight weights: torque first, then no net force
  const auto r = a.solve(w, wt, 1.0, 1e-6);
  EXPECT_NEAR(r.achieved(3), 5000, 25);
  EXPECT_NEAR(r.achieved.segment<2>(4).norm(), 0, 25);
  EXPECT_LT(r.achieved.head<3>().norm(), 100.0);  // < 0.6% m g: no parasitic lift
  // +roll: LEFT down (+z at y > 0) against RIGHT roof (-z at y < 0)
  const double left_down = r.u(0) + r.u(2), right_down = r.u(1) + r.u(3);
  const double left_roof = r.u(6) + r.u(8), right_roof = r.u(7) + r.u(9);
  EXPECT_GT(left_down, 10 * right_down - 1e-6);
  EXPECT_GT(right_roof, 10 * left_roof - 1e-6);
}

TEST(Allocator, WorldFrameForcesUseTheRoofWhenInverted)
{
  // car inverted (rolled 180 deg): world up is body -z, so a WORLD lift
  // request must come from the roof nozzles, not the (now downward) down ones
  const double mg = 1800 * 9.81;
  ThrustAllocator a(roadsterNozzles(mg), mg, mg);
  const Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Vec6 w;
  w << 0, 0, 0.5 * mg, 0, 0, 0;
  Vec6 wt;
  wt << 1, 1, 1, 1, 1, 1;
  // the node's lambda: down + roof in equal proportion is a force-free null space,
  // and only the gas penalty makes the solver leave it shut
  const auto r = a.solve(w, wt, 1.0, 1e-4, R);
  const Eigen::Vector3d f_world = R * r.achieved.head<3>();
  EXPECT_NEAR(f_world.z(), 0.5 * mg, 5e-3 * mg);
  EXPECT_NEAR(r.u.head<4>().norm(), 0.0, 1e-3);   // down nozzles shut
  EXPECT_GT(r.u.tail<4>().minCoeff(), 0.2);        // roof nozzles carry it (0.5 / 1.8 m g)
}
