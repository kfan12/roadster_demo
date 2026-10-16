// Unit tests for the pure-math actuation models (tank, nozzle, fan + gap).
#include <gtest/gtest.h>

#include <cmath>

#include "roadster_gz_plugins/fan_model.hpp"
#include "roadster_gz_plugins/nozzle_model.hpp"
#include "roadster_gz_plugins/tank_model.hpp"

using namespace roadster_gz_plugins;

TEST(Tank, NominalPointMatchesPlanCrossCheck)
{
  // 600 L at 400 bar, 293 K: plan section 2 says ~250 kg once Z ~ 1.14
  TankModel t{TankParams{}};
  EXPECT_NEAR(t.pressure(), 400e5, 1.0);
  EXPECT_NEAR(t.mass(), 250.0, 8.0);
  // ideal gas would hold ~14% more
  const double ideal = 400e5 * 0.6 / (TankModel::kRAir * 293.0);
  EXPECT_GT(ideal / t.mass(), 1.12);
}

TEST(Tank, PressureInvertsMass)
{
  TankModel t{TankParams{}};
  for (double p : {30e5, 100e5, 250e5, 399e5}) {
    t.recharge();
    t.draw(t.mass() - t.massAt(p));
    EXPECT_NEAR(t.pressure(), p, p * 1e-9);
  }
}

TEST(Tank, RegulatedFeedAndFloor)
{
  TankParams p;
  TankModel t{p};
  EXPECT_DOUBLE_EQ(t.feedPressure(), p.p_reg_pa);           // full: regulator holds
  t.draw(t.mass() - t.massAt(60e5));
  EXPECT_NEAR(t.feedPressure(), 60e5, 1.0);                  // below p_reg: blowdown
  t.draw(t.mass() - t.massAt(20e5));
  EXPECT_DOUBLE_EQ(t.feedPressure(), 0.0);                   // below p_min: no fire
  EXPECT_DOUBLE_EQ(t.draw(1e9), t.massAt(20e5));             // draw clamps at empty
  EXPECT_DOUBLE_EQ(t.mass(), 0.0);
}

TEST(Tank, BlowdownFeedIsTankPressure)
{
  TankParams p;
  p.regulated = false;
  TankModel t{p};
  EXPECT_NEAR(t.feedPressure(), 400e5, 1.0);
  EXPECT_DOUBLE_EQ(t.referencePressure(), p.p0_pa);
}

TEST(Nozzle, FirstOrderStepResponse)
{
  NozzleParams p;
  p.f_max_n = 1000.0;
  NozzleModel n{p};
  const double dt = 1e-3;
  double t = 0;
  // after one time constant the opening is 1 - 1/e
  for (int i = 0; i < 20; ++i, t += dt) {
    n.step(1.0, 1.0, 1.0, t, dt);
  }
  EXPECT_NEAR(n.opening(), 1.0 - std::exp(-1.0), 1e-3);
  for (int i = 0; i < 300; ++i, t += dt) {  // 16 time constants in total
    n.step(1.0, 1.0, 1.0, t, dt);
  }
  EXPECT_NEAR(n.thrust(), 1000.0, 1e-2);
  EXPECT_NEAR(n.massFlow(), 1000.0 / 350.0, 1e-4);
}

TEST(Nozzle, ThrustScalesWithFeedPressure)
{
  NozzleParams p;
  p.f_max_n = 1000.0;
  p.tau_rise_s = 0;
  NozzleModel n{p};
  EXPECT_DOUBLE_EQ(n.step(1.0, 50e5, 100e5, 0, 1e-3), 500.0);
  EXPECT_DOUBLE_EQ(n.step(1.0, 0.0, 100e5, 1e-3, 1e-3), 0.0);
}

TEST(Nozzle, MinimumOnTimeHoldsTheValveOpen)
{
  NozzleParams p;
  p.f_max_n = 1.0;
  p.tau_rise_s = p.tau_fall_s = 0;
  p.min_on_time_s = 0.02;
  NozzleModel n{p};
  n.step(0.7, 1, 1, 0.000, 1e-3);
  EXPECT_DOUBLE_EQ(n.step(0.0, 1, 1, 0.010, 1e-3), 0.7);  // still inside min on-time
  EXPECT_DOUBLE_EQ(n.step(0.0, 1, 1, 0.025, 1e-3), 0.0);  // released after it
}

TEST(Fan, GroundEffectAndSpinUp)
{
  FanParams p;
  p.f_max_n = 100.0;
  FanModel f{p, 1.0};
  EXPECT_DOUBLE_EQ(f.step(1.0, 0.2, 1e-3), 100.0);
  EXPECT_NEAR(f.step(1.0, 0.675, 1e-3), 50.0, 1e-9);   // halfway between seal and max
  EXPECT_DOUBLE_EQ(f.step(1.0, 1.5, 1e-3), 0.0);
  FanModel g{p, 0.0};
  for (int i = 0; i < 300; ++i) {
    g.step(1.0, 0.0, 1e-3);  // one time constant
  }
  EXPECT_NEAR(g.speed(), 1.0 - std::exp(-1.0), 1e-3);
}

TEST(Gap, GroundPlaneOnlyCountsWhenFacingIt)
{
  SurfaceSet s;
  s.planes.push_back({{0, 0, 0}, {0, 0, 1}, {-100, -100, -100}, {100, 100, 100}});
  const Vec3 up{0, 0, 1}, down{0, 0, -1};
  EXPECT_NEAR(s.gap({1, 2, 0.3}, up, M_PI / 4).h, 0.3, 1e-12);
  EXPECT_TRUE(std::isinf(s.gap({1, 2, 0.3}, down, M_PI / 4).h));  // upside down over ground
}

TEST(Gap, InvertedRampArc)
{
  // R = 8 loop centred at (0, 8): bottom at z = 0, inverted top at z = 16
  SurfaceSet s;
  s.arcs.push_back({0.0, 8.0, 8.0, -M_PI / 2, M_PI / 2, -3, 3});
  // at the top, 0.3 m below the surface, car inverted (body +z = world -z)
  const auto top = s.gap({0, 0, 15.7}, {0, 0, -1}, M_PI / 4);
  EXPECT_NEAR(top.h, 0.3, 1e-9);
  EXPECT_NEAR(top.normal.z, -1.0, 1e-9);
  // on the vertical wall at x = +8 the car's +z points to -x
  EXPECT_NEAR(s.gap({7.6, 0, 8}, {-1, 0, 0}, M_PI / 4).h, 0.4, 1e-9);
  // outside the arc's angular range (the back side, x < 0 at mid height)
  EXPECT_TRUE(std::isinf(s.gap({-7.6, 0, 8}, {1, 0, 0}, M_PI / 4).h));
  // outside its width
  EXPECT_TRUE(std::isinf(s.gap({0, 5, 15.7}, {0, 0, -1}, M_PI / 4).h));
}

TEST(Gap, AngleRangeWraps)
{
  EXPECT_TRUE(SurfaceSet::angleInRange(-M_PI / 2, -M_PI / 2, M_PI / 2));
  EXPECT_TRUE(SurfaceSet::angleInRange(0.0, -M_PI / 2, M_PI / 2));
  EXPECT_FALSE(SurfaceSet::angleInRange(M_PI, -M_PI / 2, M_PI / 2));
  EXPECT_TRUE(SurfaceSet::angleInRange(M_PI, M_PI / 2, 3 * M_PI / 2));
  EXPECT_TRUE(SurfaceSet::angleInRange(-M_PI * 0.9, M_PI / 2, 3 * M_PI / 2));
}
