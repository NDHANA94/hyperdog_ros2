// Copyright 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hyperdog_locomotion/planning/disturbance_monitor.hpp"
#include "hyperdog_locomotion/planning/foothold_planner.hpp"
#include "hyperdog_locomotion/planning/gait_scheduler.hpp"
#include "hyperdog_locomotion/planning/swing_trajectory.hpp"
#include "hyperdog_locomotion/planning/terrain_map.hpp"

using hyperdog_locomotion::DisturbanceMonitor;
using hyperdog_locomotion::FootholdParams;
using hyperdog_locomotion::GaitScheduler;
using hyperdog_locomotion::Mat43;
using hyperdog_locomotion::Vec3;
using hyperdog_locomotion::plan_foothold;
using hyperdog_locomotion::swing_trajectory;

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
  for (int k = 0; k < 130; ++k) {
    g.step(0.002);
  }
  g.request("stand");
  for (int k = 0; k < 500; ++k) {
    g.step(0.002);
  }
  EXPECT_TRUE(g.is_standing());
  for (bool c : g.contact()) {
    EXPECT_TRUE(c);
  }
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

TEST(Foothold, RaibertAndCapturePointOffsets)
{
  FootholdParams p;
  const Vec3 nominal(0.175, -0.17, 0.0);
  // standing still: foot lands right below the hip
  Vec3 f = plan_foothold(
    p, nominal, Vec3(0, 0, 0.24), 0.0, Vec3::Zero(), Vec3::Zero(), 0.0, 0.1,
    0.2, 0.24);
  EXPECT_NEAR(f.x(), 0.175, 1e-9);
  EXPECT_NEAR(f.y(), -0.17, 1e-9);
  // moving faster than commanded: step further ahead of the predicted hip (capture point
  // feedback), bounded; the hip is predicted with the measured velocity, limited to
  // v_des + max_prediction_velocity_error
  f = plan_foothold(
    p, nominal, Vec3(0, 0, 0.24), 0.0, Vec3(2.0, 0, 0), Vec3::Zero(), 0.0, 0.1, 0.2,
    0.24);
  EXPECT_NEAR(f.x() - (0.175 + p.max_prediction_velocity_error * 0.1), p.max_step_offset, 1e-9);
}

TEST(DisturbanceMonitor, DetectsPushAndSettles)
{
  DisturbanceMonitor m;
  Mat43 feet;
  feet << 0.175, -0.17, 0.02, 0.175, 0.17, 0.02, -0.175, -0.17, 0.02, -0.175, 0.17, 0.02;
  const Vec3 p(0, 0, 0.24);
  // quiet stance: nothing happens
  EXPECT_EQ(
    m.update(true, p, Vec3::Zero(), 0.24, feet, 0.0, 0.002),
    DisturbanceMonitor::Event::NONE);
  // lateral push: capture point outside the support polygon
  EXPECT_EQ(
    m.update(true, p, Vec3(0, 1.0, 0), 0.24, feet, 0.0, 0.002),
    DisturbanceMonitor::Event::DETECTED);
  EXPECT_TRUE(m.recovering());
  // settles after settle_time of rest
  DisturbanceMonitor::Event e = DisturbanceMonitor::Event::NONE;
  for (int k = 0; k < 400 && e == DisturbanceMonitor::Event::NONE; ++k) {
    e = m.update(true, p, Vec3::Zero(), 0.24, feet, 0.0, 0.002);
  }
  EXPECT_EQ(e, DisturbanceMonitor::Event::REJECTED);
  EXPECT_FALSE(m.recovering());
  // monitoring disabled while walking
  m.update(true, p, Vec3(0, 1.0, 0), 0.24, feet, 0.0, 0.002);
  m.update(false, p, Vec3(0, 1.0, 0), 0.24, feet, 0.0, 0.002);
  EXPECT_FALSE(m.recovering());
}

TEST(TerrainMap, FootholdsMoveOffStepEdgesAndTakeTheTerrainHeight)
{
  using hyperdog_locomotion::TerrainMap;
  using hyperdog_locomotion::TerrainParams;
  // 2 m x 2 m, 2 cm cells, a 4 cm step up at x = 0.5
  const int n = 100;
  std::vector<float> data(n * n);
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      const double x = -1.0 + (ix + 0.5) * 0.02;
      data[iy * n + ix] = x >= 0.5 ? 0.04f : 0.0f;
    }
  }
  TerrainMap map;
  map.set(-1.0, -1.0, 0.02, n, n, data);
  TerrainParams prm;

  // flat ground: unchanged, height from the map
  Vec3 p(0.2, 0.1, -0.3);
  ASSERT_TRUE(map.refine_foothold(prm, p));
  EXPECT_NEAR(p.x(), 0.2, 1e-9);
  EXPECT_NEAR(p.z(), 0.0, 1e-6);
  // on the edge: moved to the closest cell at least edge_radius away from it, either side
  p = Vec3(0.505, 0.1, 0.0);
  ASSERT_TRUE(map.refine_foothold(prm, p));
  EXPECT_GT(std::abs(p.x() - 0.5), prm.edge_radius);
  EXPECT_LT(std::abs(p.x() - 0.5), prm.edge_radius + 0.05);
  EXPECT_NEAR(p.z(), p.x() > 0.5 ? 0.04 : 0.0, 1e-6);
  EXPECT_LT(map.roughness(p.x(), p.y(), prm.edge_radius), prm.edge_threshold);
  // outside the map: not refined
  p = Vec3(3.0, 0.0, -0.5);
  EXPECT_FALSE(map.refine_foothold(prm, p));
  EXPECT_DOUBLE_EQ(p.z(), -0.5);
  // malformed map: ignored
  map.set(0.0, 0.0, 0.02, 3, 3, std::vector<float>(5, 0.0f));
  EXPECT_TRUE(map.empty());
}

TEST(SwingTrajectory, StepOverLiftsBeforeMovingAndClearsTheApex)
{
  using hyperdog_locomotion::SwingShape;
  const Vec3 p0(0.0, 0.0, 0.02), pf(0.15, 0.0, 0.06);
  SwingShape shape;
  shape.apex_z = 0.14;
  shape.step_over = true;
  const double T = 0.2;
  // lift first: no horizontal motion yet, already well above the start
  auto a = swing_trajectory(p0, pf, 0.06, 0.2, T, 0.01, shape);
  EXPECT_NEAR(a.pos.x(), 0.0, 1e-9);
  EXPECT_GT(a.pos.z(), 0.08);
  // horizontal motion done while still near the apex
  auto b = swing_trajectory(p0, pf, 0.06, 0.8, T, 0.01, shape);
  EXPECT_NEAR(b.pos.x(), 0.15, 1e-9);
  EXPECT_GT(b.pos.z(), 0.13);
  auto m = swing_trajectory(p0, pf, 0.06, 0.5, T, 0.01, shape);
  EXPECT_NEAR(m.pos.z(), 0.14, 1e-9);
  auto e = swing_trajectory(p0, pf, 0.06, 1.0, T, 0.01, shape);
  EXPECT_NEAR(e.pos.z(), 0.05, 1e-9);
  EXPECT_NEAR(e.vel.norm(), 0.0, 1e-9);
  // velocity is the derivative of the position
  const double s = 0.5, ds = 1e-6;
  auto p1 = swing_trajectory(p0, pf, 0.06, s - ds, T, 0.01, shape);
  auto p2 = swing_trajectory(p0, pf, 0.06, s + ds, T, 0.01, shape);
  EXPECT_LT(((p2.pos - p1.pos) / (2 * ds * T) - m.vel).norm(), 1e-4);
}

TEST(TerrainMap, RegistrationFromFootContacts)
{
  using hyperdog_locomotion::TerrainMap;
  using hyperdog_locomotion::TerrainParams;
  // map: 4 cm step at x = 0.5; the odometry has drifted: the step is at x = 0.45 for the robot
  const int n = 100;
  std::vector<float> data(n * n);
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      data[iy * n + ix] = -1.0 + (ix + 0.5) * 0.02 >= 0.5 ? 0.04f : 0.0f;
    }
  }
  TerrainMap map;
  map.set(-1.0, -1.0, 0.02, n, n, data);
  std::vector<Vec3> feet;
  for (double x : {0.30, 0.36, 0.42, 0.43}) {
    feet.emplace_back(x, 0.1, 0.0);
  }
  for (double x : {0.465, 0.52, 0.58, 0.64}) {
    feet.emplace_back(x, -0.1, 0.04);
  }
  TerrainParams prm;
  Vec3 o = Vec3::Zero();
  for (int k = 0; k < 5; ++k) {
    o = map.estimate_offset(feet, o, prm);
  }                                                                      // rate limited
  EXPECT_NEAR(o.x(), 0.05, 0.025);
  EXPECT_NEAR(o.y(), 0.0, 0.025);
  EXPECT_NEAR(o.z(), 0.0, 1e-6);
  map.set_offset(o);
  double h;
  ASSERT_TRUE(map.height(0.43, 0.0, 0.005, h));
  EXPECT_NEAR(h, 0.0, 1e-6);
  ASSERT_TRUE(map.height(0.47, 0.0, 0.005, h));
  EXPECT_NEAR(h, 0.04, 1e-6);
  // too few samples: prior kept
  feet.resize(3);
  EXPECT_LT(
    (map.estimate_offset(feet, Vec3(0.01, 0.0, 0.0), prm) - Vec3(0.01, 0.0, 0.0)).norm(),
    1e-12);
}
