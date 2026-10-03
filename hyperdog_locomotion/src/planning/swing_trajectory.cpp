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
#include <cmath>

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

/// Min-jerk transition over the part [a, b] of the swing phase s (constant outside).
void segment(double s, double a, double b, double duration, double & p, double & v, double & acc)
{
  const double T = (b - a) * duration;
  min_jerk((s - a) / (b - a), p, v, acc);
  if (s < a || s > b) {v = acc = 0.0;}
  v /= T;
  acc /= T * T;
}
}  // namespace

SwingSample swing_trajectory(
  const Vec3 & p0, const Vec3 & pf, double height, double s, double duration,
  double touchdown_depth, const SwingShape & shape)
{
  SwingSample out;
  duration = std::max(duration, 1e-3);
  const double apex = std::isnan(shape.apex_z) ? std::max(p0.z(), pf.z()) + height : shape.apex_z;
  const double z_end = pf.z() - touchdown_depth;
  // phase windows: horizontal motion [xy0, xy1], lift [0, up], lower [down, 1]
  const double xy0 = shape.step_over ? 0.2 : 0.0, xy1 = shape.step_over ? 0.8 : 1.0;
  const double up = shape.step_over ? 0.35 : 0.5, down = shape.step_over ? 0.75 : 0.5;
  double b, db, ddb;
  segment(s, xy0, xy1, duration, b, db, ddb);
  const Eigen::Vector2d d = pf.head<2>() - p0.head<2>();
  out.pos.head<2>() = p0.head<2>() + d * b;
  out.vel.head<2>() = d * db;
  out.acc.head<2>() = d * ddb;
  if (s < up) {
    segment(s, 0.0, up, duration, b, db, ddb);
    out.pos.z() = p0.z() + (apex - p0.z()) * b;
    out.vel.z() = (apex - p0.z()) * db;
    out.acc.z() = (apex - p0.z()) * ddb;
  } else if (s < down) {
    out.pos.z() = apex;
  } else {
    segment(s, down, 1.0, duration, b, db, ddb);
    out.pos.z() = apex + (z_end - apex) * b;
    out.vel.z() = (z_end - apex) * db;
    out.acc.z() = (z_end - apex) * ddb;
  }
  return out;
}

}  // namespace hyperdog_locomotion
