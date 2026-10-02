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

#include "hyperdog_locomotion/estimation/kinematic_kalman_filter.hpp"

namespace hyperdog_locomotion
{

KinematicKalmanFilter::KinematicKalmanFilter(const KalmanParams & p, double dt)
: p_(p), dt_(dt)
{
  A_.setIdentity();
  A_.block<3, 3>(0, 3) = dt * Mat3::Identity();
  B_.setZero();
  B_.block<3, 3>(3, 0) = dt * Mat3::Identity();
  C_.setZero();
  for (int i = 0; i < 4; ++i) {
    C_.block<3, 3>(3 * i, 0) = Mat3::Identity();
    C_.block<3, 3>(3 * i, 6 + 3 * i) = -Mat3::Identity();
    C_.block<3, 3>(12 + 3 * i, 3) = Mat3::Identity();
    C_(24 + i, 8 + 3 * i) = 1.0;
  }
}

void KinematicKalmanFilter::reset(const Vec3 & base_pos, const Mat43 & feet_world)
{
  x_.setZero();
  x_.segment<3>(0) = base_pos;
  for (int i = 0; i < 4; ++i) {
    x_.segment<3>(6 + 3 * i) = feet_world.row(i).transpose();
  }
  P_ = Mat18::Identity() * 0.01;
}

void KinematicKalmanFilter::update(
  const Mat3 & R, const Vec3 & omega_body, const Vec3 & accel_body,
  const Mat43 & feet_body, const Mat43 & feet_vel_body, const std::array<double, 4> & trust,
  const std::array<double, 4> & foot_ground_z)
{
  const double dt = dt_;
  const Vec3 a_world = R * accel_body - Vec3(0.0, 0.0, kGravity);
  // predict
  x_ = A_ * x_ + B_ * a_world;
  Mat18 Q = Mat18::Zero();
  Q.block<3, 3>(0, 0) = Mat3::Identity() * p_.process_noise_position * dt;
  Q.block<3, 3>(3, 3) = Mat3::Identity() * p_.process_noise_velocity * dt * kGravity;
  for (int i = 0; i < 4; ++i) {
    const double scale = 1.0 + (1.0 - trust[i]) * p_.swing_noise_scale;
    Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = Mat3::Identity() * p_.process_noise_foot * dt * scale;
  }
  P_ = A_ * P_ * A_.transpose() + Q;

  // correct
  Eigen::Matrix<double, 28, 1> y;
  Eigen::Matrix<double, 28, 1> r_diag;
  const Vec3 omega_w = R * omega_body;
  for (int i = 0; i < 4; ++i) {
    const Vec3 p_rel = R * feet_body.row(i).transpose();
    const Vec3 v_rel = R * feet_vel_body.row(i).transpose() + omega_w.cross(p_rel);
    const double scale = 1.0 + (1.0 - trust[i]) * p_.swing_noise_scale;
    y.segment<3>(3 * i) = -p_rel;
    y.segment<3>(12 + 3 * i) = -v_rel;
    y(24 + i) = foot_ground_z[i];
    r_diag.segment<3>(3 * i).setConstant(p_.measurement_noise_position);
    r_diag.segment<3>(12 + 3 * i).setConstant(p_.measurement_noise_velocity * scale);
    r_diag(24 + i) = p_.measurement_noise_foot_height * scale;
  }
  const Eigen::Matrix<double, 28, 28> S = C_ * P_ * C_.transpose() +
    Eigen::Matrix<double, 28, 28>(r_diag.asDiagonal());
  const Eigen::Matrix<double, 18, 28> K =
    S.ldlt().solve(C_ * P_).transpose();
  x_ += K * (y - C_ * x_);
  P_ = (Mat18::Identity() - K * C_) * P_;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

}  // namespace hyperdog_locomotion
