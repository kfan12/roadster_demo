// Unit tests for the Demo B hop planner (hop_planner.hpp).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "roadster_control/hop_planner.hpp"

using roadster_control::HopObstacle;
using roadster_control::HopParams;
using roadster_control::hopRequiredHeight;
using roadster_control::planHop;

TEST(HopPlanner, OneBoxPutsTheApexOverItsMiddle)
{
  HopParams p;
  const std::vector<HopObstacle> row{{10.0, 12.0, 4.0}};
  const auto plan = planHop(row, p, 1800.0);
  ASSERT_TRUE(plan.ok);
  EXPECT_NEAR(plan.apex_x, 11.0, 0.03);
  // car half length beyond either side: the interval is 11 +- 3.3
  const double k = p.gravity / (2 * p.speed * p.speed);
  EXPECT_NEAR(plan.apex_height, hopRequiredHeight(row[0], p) + k * 3.3 * 3.3, 0.01);
  EXPECT_NEAR(plan.clearance, p.clearance, 0.005);
}

TEST(HopPlanner, BurnThenBallisticArcClearsEveryBox)
{
  // the Demo B row: Model 3, Model Y, Semi, Cybertruck, Cybercab
  HopParams p;
  const std::vector<HopObstacle> row{
    {0.0, 1.85, 1.44}, {2.35, 4.27, 1.62}, {4.77, 7.37, 4.1}, {7.87, 10.07, 1.79},
    {10.57, 12.47, 1.4}};
  const double m = 1800.0;
  const auto plan = planHop(row, p, m);
  ASSERT_TRUE(plan.ok);
  EXPECT_GT(plan.apex_x, 4.77);  // the Semi sets the apex
  EXPECT_LT(plan.apex_x, 7.37);
  // fly the plan: burn at a for T from standing at burn_start_x, then ballistic
  const double dt = 1e-4;
  double x = plan.burn_start_x, h = 0.0, vz = 0.0, t = 0.0, h_max = 0.0;
  std::vector<double> min_clear(row.size(), 1e9);
  while (h >= 0.0 && t < 10.0) {
    const double az = t < plan.burn_time ? p.burn_accel : -p.gravity;
    vz += az * dt;
    h += vz * dt;
    x += p.speed * dt;
    t += dt;
    h_max = std::max(h_max, h);
    for (std::size_t i = 0; i < row.size(); ++i) {
      if (x >= row[i].x0 - p.half_length && x <= row[i].x1 + p.half_length) {
        const double bottom = h + p.com_rest_height - p.bottom_below_com;
        min_clear[i] = std::min(min_clear[i], bottom - row[i].top);
      }
    }
  }
  EXPECT_NEAR(h_max, plan.apex_height, 0.01);
  for (std::size_t i = 0; i < row.size(); ++i) {
    EXPECT_GT(min_clear[i], p.clearance - 2e-3) << "box " << i;
  }
  EXPECT_NEAR(*std::min_element(min_clear.begin(), min_clear.end()), p.clearance, 0.01);
  // impulse: up burn m (g + a) T
  EXPECT_NEAR(plan.up_impulse, m * (p.gravity + p.burn_accel) * plan.burn_time, 1e-6);
  EXPECT_GT(plan.landing_impulse, m * std::sqrt(2 * p.gravity * plan.apex_height));
}

TEST(HopPlanner, FlipPlanClearsTheRolledCarAndEndsBeforeTheLandingBurn)
{
  HopParams p;
  p.flip_turns = 1.0;
  p.flip_duration = 1.4;
  const std::vector<HopObstacle> row{
    {0.0, 1.85, 1.44}, {2.35, 4.27, 1.62}, {4.77, 7.37, 4.1}, {7.87, 10.07, 1.79},
    {10.57, 12.47, 1.4}};
  const double m = 1800.0;
  HopParams level = p;
  level.flip_turns = 0.0;
  const auto flat = planHop(row, level, m);
  const auto plan = planHop(row, p, m);
  ASSERT_TRUE(plan.ok);
  EXPECT_GT(plan.max_depth, p.bottom_below_com + 0.1);
  EXPECT_GT(plan.apex_height, flat.apex_height);
  // the roll's torque peaks lift: the burn needs less than the level hop's
  // despite the deeper rolled car
  EXPECT_LT(plan.burn_time, flat.burn_time);
  EXPECT_GT(plan.flip_impulse, 0.0);
  EXPECT_LT(plan.flip_end_time + 0.1, plan.brake_latch_time);
  // fly it: burn, then the roll's vertical thrust on top of gravity; the
  // rolled cross-section must keep the clearance over every box
  const double dt = 1e-4, theta = 2 * M_PI;
  double x = plan.burn_start_x, h = 0.0, vz = 0.0, t = 0.0;
  std::vector<double> min_clear(row.size(), 1e9);
  while (h >= 0.0 && t < 10.0) {
    double az = -p.gravity, phi = 0.0;
    if (t < plan.burn_time) {
      az = p.burn_accel;
    } else {
      double dds = 0.0;
      const double u = (t - plan.burn_time) / p.flip_duration;
      phi = theta * roadster_control::hopMinJerk(u, &dds);
      if (u < 1.0) {
        const double F = std::abs(p.roll_inertia * theta * dds /
            (p.flip_duration * p.flip_duration)) / p.roll_lever;
        az += F * std::cos(phi) / m;
      }
    }
    vz += az * dt;
    h += vz * dt;
    x += p.speed * dt;
    t += dt;
    for (std::size_t i = 0; i < row.size(); ++i) {
      if (x >= row[i].x0 - p.half_length && x <= row[i].x1 + p.half_length) {
        const double low = h + p.com_rest_height - roadster_control::hopRollDepth(p, phi);
        min_clear[i] = std::min(min_clear[i], low - row[i].top);
      }
    }
  }
  for (std::size_t i = 0; i < row.size(); ++i) {
    EXPECT_GT(min_clear[i], p.clearance - 0.01) << "box " << i;
  }
}

TEST(HopPlanner, RollDepthUprightIsTheWheelsAndInvertedIsTheRoof)
{
  HopParams p;
  EXPECT_NEAR(roadster_control::hopRollDepth(p, 0.0), p.bottom_below_com, 1e-12);
  EXPECT_NEAR(roadster_control::hopRollDepth(p, M_PI), p.roof_above_com, 1e-12);
  EXPECT_NEAR(roadster_control::hopRollDepth(p, M_PI / 2), p.half_width, 1e-12);
}

TEST(HopPlanner, FasterApproachNeedsALowerApex)
{
  HopParams slow, fast;
  slow.speed = 10.0;
  fast.speed = 20.0;
  const std::vector<HopObstacle> row{{0.0, 2.6, 4.1}};
  EXPECT_LT(planHop(row, fast, 1800).apex_height, planHop(row, slow, 1800).apex_height);
  EXPECT_FALSE(planHop({}, fast, 1800).ok);
}
