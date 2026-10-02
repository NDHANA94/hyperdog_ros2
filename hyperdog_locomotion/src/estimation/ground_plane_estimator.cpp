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

#include "hyperdog_locomotion/estimation/ground_plane_estimator.hpp"

namespace hyperdog_locomotion
{

void GroundPlaneEstimator::reset(const Mat43 & feet_world, double foot_radius)
{
  hist_ = feet_world;
  const double z = feet_world.col(2).mean() - foot_radius;
  coef_ = Vec3(z, 0.0, 0.0);
}

void GroundPlaneEstimator::update(const Mat43 & feet_world, const Bool4 & use, double foot_radius)
{
  for (int i = 0; i < 4; ++i) {
    if (use[i]) {hist_.row(i) = feet_world.row(i);}
  }
  Eigen::Matrix<double, 4, 3> A;
  A.col(0).setOnes();
  A.col(1) = hist_.col(0);
  A.col(2) = hist_.col(1);
  const Eigen::Vector4d b = hist_.col(2).array() - foot_radius;
  const Vec3 c = A.colPivHouseholderQr().solve(b);
  if (c.allFinite()) {coef_ += alpha_ * (c - coef_);}
}

Eigen::Vector2d GroundPlaneEstimator::slope_rp(double yaw) const
{
  const double b = coef_[1], c = coef_[2];
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double along = b * cy + c * sy;      // slope along the heading
  const double side = -b * sy + c * cy;      // slope sideways
  return Eigen::Vector2d(std::atan(side), -std::atan(along));
}

}  // namespace hyperdog_locomotion
