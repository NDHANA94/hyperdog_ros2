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

#include "hyperdog_locomotion/planning/swing_trajectory.hpp"

#include <algorithm>

namespace hyperdog_locomotion
{

namespace
{
void min_jerk(double s, double & p, double & v, double & a)
{
  s = std::clamp(s, 0.0, 1.0);
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  p = 10 * s3 - 15 * s4 + 6 * s5;
  v = 30 * s2 - 60 * s3 + 30 * s4;
  a = 60 * s - 180 * s2 + 120 * s3;
}
}  // namespace

SwingSample swing_trajectory(
  const Vec3 & p0, const Vec3 & pf, double height, double s, double duration,
  double touchdown_depth)
{
  SwingSample out;
  duration = std::max(duration, 1e-3);
  double b, db, ddb;
  min_jerk(s, b, db, ddb);
  const Eigen::Vector2d d = pf.head<2>() - p0.head<2>();
  out.pos.head<2>() = p0.head<2>() + d * b;
  out.vel.head<2>() = d * db / duration;
  out.acc.head<2>() = d * ddb / (duration * duration);
  const double apex = std::max(p0.z(), pf.z()) + height;
  const double z_end = pf.z() - touchdown_depth;
  if (s < 0.5) {
    min_jerk(2.0 * s, b, db, ddb);
    out.pos.z() = p0.z() + (apex - p0.z()) * b;
    out.vel.z() = (apex - p0.z()) * db * 2.0 / duration;
    out.acc.z() = (apex - p0.z()) * ddb * 4.0 / (duration * duration);
  } else {
    min_jerk(2.0 * s - 1.0, b, db, ddb);
    out.pos.z() = apex + (z_end - apex) * b;
    out.vel.z() = (z_end - apex) * db * 2.0 / duration;
    out.acc.z() = (z_end - apex) * ddb * 4.0 / (duration * duration);
  }
  return out;
}

}  // namespace hyperdog_locomotion
