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

#include "hyperdog_locomotion/estimation/attitude_filter.hpp"
#include "hyperdog_locomotion/estimation/contact_estimator.hpp"
#include "hyperdog_locomotion/estimation/ground_plane_estimator.hpp"
#include "hyperdog_locomotion/estimation/kinematic_kalman_filter.hpp"

using hyperdog_locomotion::Bool4;
using hyperdog_locomotion::ContactEstimator;
using hyperdog_locomotion::GroundPlaneEstimator;
using hyperdog_locomotion::KalmanParams;
using hyperdog_locomotion::KinematicKalmanFilter;
using hyperdog_locomotion::MahonyFilter;
using hyperdog_locomotion::Mat3;
using hyperdog_locomotion::Mat43;
using hyperdog_locomotion::Vec3;
using hyperdog_locomotion::kGravity;
using hyperdog_locomotion::rot_to_rpy;
using hyperdog_locomotion::rpy_to_rot;

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
    kf.update(
      Mat3::Identity(), Vec3::Zero(), Vec3(0.05, 0, kGravity), fb, Mat43::Zero(), trust,
      gz);
  }
  EXPECT_LT(kf.velocity().norm(), 0.01);
  EXPECT_NEAR(kf.position().z(), 0.24, 0.01);
}

TEST(GroundPlane, EstimatesSlope)
{
  GroundPlaneEstimator g(1.0);
  Mat43 f;
  // 10 % uphill in +x
  f << 0.2, -0.2, 0.02 + 0.02, 0.2, 0.2, 0.02 + 0.02, -0.2, -0.2, 0.02 - 0.02, -0.2, 0.2,
    0.02 - 0.02;
  g.update(f, Bool4{true, true, true, true}, 0.02);
  EXPECT_NEAR(g.slope_rp(0.0).y(), -std::atan(0.1), 1e-6);
  EXPECT_NEAR(g.slope_rp(0.0).x(), 0.0, 1e-6);
}

TEST(ContactEstimator, EarlyAndLateTouchdown)
{
  ContactEstimator c;
  const Bool4 sched{true, false, false, true};
  const Bool4 sensor{false, true, false, true};
  c.update(sched, {0.1, 0.8, 0.3, 0.5}, &sensor, nullptr);
  EXPECT_TRUE(c.late()[0]);      // scheduled stance, no contact yet
  EXPECT_FALSE(c.contact()[0]);
  EXPECT_TRUE(c.early()[1]);     // contact in late swing
  EXPECT_TRUE(c.contact()[1]);
  EXPECT_FALSE(c.contact()[2]);  // swinging
  EXPECT_TRUE(c.contact()[3]);
}

TEST(MahonyFilter, ConvergesToGravityDirection)
{
  MahonyFilter f(2.0, 0.0);
  // robot rolled by 0.2 rad: gravity seen in the body frame
  const Vec3 accel = rpy_to_rot(Vec3(0.2, 0.0, 0.0)).transpose() * Vec3(0, 0, kGravity);
  Eigen::Quaterniond q;
  for (int k = 0; k < 5000; ++k) {
    q = f.update(Vec3::Zero(), accel, 0.002);
  }
  EXPECT_NEAR(rot_to_rpy(q.toRotationMatrix()).x(), 0.2, 1e-3);
}
