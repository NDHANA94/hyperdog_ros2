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

#include "hyperdog_locomotion/planning/disturbance_monitor.hpp"
#include "hyperdog_locomotion/planning/foothold_planner.hpp"
#include "hyperdog_locomotion/planning/gait_scheduler.hpp"
#include "hyperdog_locomotion/planning/swing_trajectory.hpp"

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
  // moving faster than commanded: step further ahead (capture point feedback), bounded
  f = plan_foothold(
    p, nominal, Vec3(0, 0, 0.24), 0.0, Vec3(2.0, 0, 0), Vec3::Zero(), 0.0, 0.1, 0.2,
    0.24);
  EXPECT_NEAR(f.x() - 0.175, p.max_step_offset, 1e-9);
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
