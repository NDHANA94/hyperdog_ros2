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
// Minimum-jerk swing foot trajectories.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct SwingSample
{
  Vec3 pos{Vec3::Zero()};
  Vec3 vel{Vec3::Zero()};
  Vec3 acc{Vec3::Zero()};
};

/// Min-jerk swing from p0 to pf with an apex `height` above the higher end point.
/// The trajectory ends `touchdown_depth` below pf to guarantee ground contact.
SwingSample swing_trajectory(
  const Vec3 & p0, const Vec3 & pf, double height, double s, double duration,
  double touchdown_depth);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_
