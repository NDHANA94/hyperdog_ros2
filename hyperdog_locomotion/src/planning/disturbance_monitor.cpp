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

#include "hyperdog_locomotion/planning/disturbance_monitor.hpp"

#include <algorithm>

namespace hyperdog_locomotion
{

DisturbanceMonitor::Event DisturbanceMonitor::update(
  bool active, const Vec3 & base_pos, const Vec3 & base_vel, double base_height,
  const Mat43 & feet_world, double tilt, double dt)
{
  if (!p_.enabled || !active) {
    reset();
    return Event::NONE;
  }
  const double h = std::max(base_height, 0.1);
  const Eigen::Vector2d cp = base_pos.head<2>() + base_vel.head<2>() * std::sqrt(h / kGravity);
  const Eigen::Vector2d lo =
    feet_world.leftCols<2>().colwise().minCoeff().transpose().array() + p_.capture_point_margin;
  const Eigen::Vector2d hi =
    feet_world.leftCols<2>().colwise().maxCoeff().transpose().array() - p_.capture_point_margin;
  const bool outside = (cp.array() < lo.array()).any() || (cp.array() > hi.array()).any();
  const double speed = base_vel.head<2>().norm();

  Event event = Event::NONE;
  if (!recovering_ &&
    (outside || speed > p_.velocity_threshold || tilt > p_.tilt_threshold))
  {
    recovering_ = true;
    settle_timer_ = 0.0;
    event = Event::DETECTED;
  }
  if (recovering_) {
    const bool settled = speed < p_.settle_velocity && tilt < 0.5 * p_.tilt_threshold && !outside;
    settle_timer_ = settled ? settle_timer_ + dt : 0.0;
    if (settle_timer_ > p_.settle_time) {
      recovering_ = false;
      event = Event::REJECTED;
    }
  }
  return event;
}

}  // namespace hyperdog_locomotion
