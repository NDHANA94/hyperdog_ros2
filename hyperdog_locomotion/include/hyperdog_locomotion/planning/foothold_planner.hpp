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
// Foot placement: Raibert heuristic + capture point feedback + centrifugal term.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__FOOTHOLD_PLANNER_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__FOOTHOLD_PLANNER_HPP_

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct FootholdParams
{
  double capture_point_gain{1.0};
  double centrifugal_gain{0.5};
  double max_step_offset{0.12};
};

/// p_hip(t_td) is predicted with the measured velocity (v_world) and the commanded yaw rate.
/// p_td = p_hip(t_td) + v T_st/2 + k_cp sqrt(h/g) (v - v_des) + k_c h/g (v x w_des)
Vec3 plan_foothold(
  const FootholdParams & prm, const Vec3 & nominal_body, const Vec3 & base_pos, double yaw,
  const Vec3 & v_world, const Vec3 & v_des_world, double yaw_rate_des, double t_remaining,
  double t_stance, double height);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__FOOTHOLD_PLANNER_HPP_
