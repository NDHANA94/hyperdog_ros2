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
//
// Types shared by the ground reaction force controllers: rigid body state,
// contact force limits and friction pyramid constraints.

#ifndef HYPERDOG_LOCOMOTION__CONTROL__FORCE_CONTROL_COMMON_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__FORCE_CONTROL_COMMON_HPP_

#include <Eigen/Dense>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct BodyState
{
  Mat3 R{Mat3::Identity()};
  Vec3 rpy{Vec3::Zero()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
  Vec3 omega_world{Vec3::Zero()};
};

struct ContactLimits
{
  double mu{0.6};
  double fz_min{3.0};
  double fz_max{160.0};
};

/// Friction pyramid rows for 4 feet (12 forces). Swing feet are constrained to zero.
void friction_constraints(
  const Bool4 & contact, const ContactLimits & lim, const Vec3 & normal,
  Eigen::MatrixXd & A, Eigen::VectorXd & l, Eigen::VectorXd & u, int col_offset = 0,
  int total_cols = 12);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__FORCE_CONTROL_COMMON_HPP_
