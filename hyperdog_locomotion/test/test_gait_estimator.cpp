// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
#include <gtest/gtest.h>

#include "hyperdog_locomotion/estimator.hpp"
#include "hyperdog_locomotion/gait_scheduler.hpp"
#include "hyperdog_locomotion/swing.hpp"

using namespace hyperdog_locomotion;

TEST(Gait, TrotHasDiagonalPairs)
{
  GaitScheduler g;
  g.request("trot");
  for (int k = 0; k < 500; ++k) {
    g.step(0.002);
    const auto & c = g.contact();
    EXPECT_EQ(c[0], c[3]);
    EXPECT_EQ(c[1], c[2]);
    EXPECT_TRUE(c[0] || c[1]);   // duty > 0.5: never all feet in the air
  }
}

TEST(Gait, StopsWithAllFeetDown)
{
  GaitScheduler g;
  g.request("trot");
  for (int k = 0; k < 130; ++k) {g.step(0.002);}
  g.request("stand");
  for (int k = 0; k < 500; ++k) {g.step(0.002);}
  EXPECT_TRUE(g.is_standing());
  for (bool c : g.contact()) {EXPECT_TRUE(c);}
}

TEST(Swing, StartsAndEndsAtTargets)
{
  const Vec3 p0(0, 0, 0.02), pf(0.1, 0.02, 0.02);
  auto a = swing_trajectory(p0, pf, 0.06, 0.0, 0.2, 0.01);
  auto m = swing_trajectory(p0, pf, 0.06, 0.5, 0.2, 0.01);
  auto b = swing_trajectory(p0, pf, 0.06, 1.0, 0.2, 0.01);
  EXPECT_LT((a.pos - p0).norm(), 1e-9);
  EXPECT_NEAR(m.pos.z(), 0.08, 1e-9);
  EXPECT_NEAR(b.pos.x(), 0.1, 1e-9);
  EXPECT_NEAR(b.pos.z(), 0.01, 1e-9);
  EXPECT_LT(b.vel.norm(), 1e-9);
}

TEST(KalmanFilter, StandingRobotStaysPut)
{
  KinematicKalmanFilter kf(KalmanParams(), 0.002);
  Mat43 fb;
  fb << 0.175, -0.17, -0.22, 0.175, 0.17, -0.22, -0.175, -0.17, -0.22, -0.175, 0.17, -0.22;
  Mat43 fw = fb;
  fw.col(2).array() += 0.24;
  kf.reset(Vec3(0, 0, 0.24), fw);
  const std::array<double, 4> trust{1, 1, 1, 1}, gz{0.02, 0.02, 0.02, 0.02};
  for (int k = 0; k < 1000; ++k) {
    kf.update(Mat3::Identity(), Vec3::Zero(), Vec3(0.05, 0, kGravity), fb, Mat43::Zero(), trust, gz);
  }
  EXPECT_LT(kf.velocity().norm(), 0.01);
  EXPECT_NEAR(kf.position().z(), 0.24, 0.01);
}

TEST(GroundPlane, EstimatesSlope)
{
  GroundPlaneEstimator g(1.0);
  Mat43 f;
  // 10 % uphill in +x
  f << 0.2, -0.2, 0.02 + 0.02, 0.2, 0.2, 0.02 + 0.02, -0.2, -0.2, 0.02 - 0.02, -0.2, 0.2, 0.02 - 0.02;
  g.update(f, Bool4{true, true, true, true}, 0.02);
  EXPECT_NEAR(g.slope_rp(0.0).y(), -std::atan(0.1), 1e-6);
  EXPECT_NEAR(g.slope_rp(0.0).x(), 0.0, 1e-6);
}
