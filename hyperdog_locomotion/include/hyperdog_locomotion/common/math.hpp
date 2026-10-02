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
// Common types and rotation / linear algebra helpers.

#ifndef HYPERDOG_LOCOMOTION__COMMON__MATH_HPP_
#define HYPERDOG_LOCOMOTION__COMMON__MATH_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

namespace hyperdog_locomotion
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Mat43 = Eigen::Matrix<double, 4, 3>;   // one row per leg
using Bool4 = std::array<bool, 4>;

constexpr double kGravity = 9.81;

inline Mat3 skew(const Vec3 & v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

/// R = Rz(yaw) Ry(pitch) Rx(roll)
inline Mat3 rpy_to_rot(const Vec3 & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Vec3::UnitX())).toRotationMatrix();
}

inline Vec3 rot_to_rpy(const Mat3 & R)
{
  return Vec3(
    std::atan2(R(2, 1), R(2, 2)),
    -std::asin(std::clamp(R(2, 0), -1.0, 1.0)),
    std::atan2(R(1, 0), R(0, 0)));
}

inline Mat3 rot_z(double yaw)
{
  return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
}

/// Rotation vector (axis * angle) of R.
inline Vec3 so3_log(const Mat3 & R)
{
  Eigen::AngleAxisd aa(R);
  return aa.axis() * aa.angle();
}

inline double wrap_angle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {a += 2.0 * M_PI;}
  return a - M_PI;
}

inline double smoothstep_cos(double a)
{
  a = std::clamp(a, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * a);
}

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__COMMON__MATH_HPP_
