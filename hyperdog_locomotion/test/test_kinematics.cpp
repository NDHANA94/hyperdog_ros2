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

#include <random>

#include "hyperdog_locomotion/kinematics/leg_kinematics.hpp"

using hyperdog_locomotion::Mat3;
using hyperdog_locomotion::RobotKinematics;
using hyperdog_locomotion::Vec3;

TEST(Kinematics, NominalStandingPose)
{
  RobotKinematics k;
  Vec3 q;
  ASSERT_TRUE(k.leg(0).inverse(k.leg(0).nominal_foot(0.22), q));
  EXPECT_NEAR(q[0], 0.0, 1e-9);
  EXPECT_NEAR(q[1], 0.8903, 1e-3);
  EXPECT_NEAR(q[2], 1.7214, 1e-3);
  // right / left legs
  EXPECT_NEAR(k.leg(0).nominal_foot(0.2).y(), -0.170, 1e-9);
  EXPECT_NEAR(k.leg(1).nominal_foot(0.2).y(), 0.170, 1e-9);
}

TEST(Kinematics, InverseOfForwardAndJacobian)
{
  RobotKinematics k;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> h(-0.6, 0.6), t(-0.5, 2.5), c(0.5, 2.3);
  for (int leg = 0; leg < 4; ++leg) {
    for (int n = 0; n < 500; ++n) {
      const Vec3 q(h(rng), t(rng), c(rng));
      const Vec3 p = k.leg(leg).forward(q);
      // the IK solves the knee-down branch: only test feet below the hip
      if (p.z() > -0.05) {continue;}
      Vec3 q2;
      k.leg(leg).inverse(p, q2);
      EXPECT_LT((k.leg(leg).forward(q2) - p).norm(), 1e-4);
      const Mat3 J = k.leg(leg).jacobian(q);
      for (int i = 0; i < 3; ++i) {
        Vec3 dq = Vec3::Zero();
        dq[i] = 1e-6;
        const Vec3 num = (k.leg(leg).forward(q + dq) - p) / 1e-6;
        EXPECT_LT((J.col(i) - num).norm(), 1e-4);
      }
    }
  }
}
