// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
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

/// p_td = p_hip(t_td) + v T_st/2 + k_cp sqrt(h/g) (v - v_des) + k_c h/g (v x w_des)
Vec3 plan_foothold(
  const FootholdParams & prm, const Vec3 & nominal_body, const Vec3 & base_pos, double yaw,
  const Vec3 & v_world, const Vec3 & v_des_world, double yaw_rate_des, double t_remaining,
  double t_stance, double height);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__FOOTHOLD_PLANNER_HPP_
