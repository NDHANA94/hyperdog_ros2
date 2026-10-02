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
// Push detection for the standing robot. When the instantaneous capture point
// leaves the support polygon (minus a margin), or the body moves / tilts too
// much, the robot has to step to recover: the monitor latches "recovering"
// until the body has settled for a while.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__DISTURBANCE_MONITOR_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__DISTURBANCE_MONITOR_HPP_

#include <string>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct DisturbanceRecoveryParams
{
  bool enabled{true};
  double velocity_threshold{0.25};    // [m/s] body speed that triggers stepping
  double tilt_threshold{0.2};         // [rad] roll / pitch error that triggers stepping
  double capture_point_margin{0.03};  // [m]
  double settle_velocity{0.06};       // [m/s]
  double settle_time{0.6};            // [s]
  std::string gait{"trot"};           // gait used for recovery steps
};

class DisturbanceMonitor
{
public:
  enum class Event {NONE, DETECTED, REJECTED};

  explicit DisturbanceMonitor(const DisturbanceRecoveryParams & p = DisturbanceRecoveryParams())
  : p_(p) {}

  /// active: monitoring allowed (robot standing, no commanded motion).
  /// base_height: base height above the ground, tilt: attitude error w.r.t. the terrain.
  Event update(
    bool active, const Vec3 & base_pos, const Vec3 & base_vel, double base_height,
    const Mat43 & feet_world, double tilt, double dt);
  void reset() {recovering_ = false; settle_timer_ = 0.0;}
  bool recovering() const {return recovering_;}
  const DisturbanceRecoveryParams & params() const {return p_;}

private:
  DisturbanceRecoveryParams p_;
  bool recovering_{false};
  double settle_timer_{0.0};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__DISTURBANCE_MONITOR_HPP_
