// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
// Foothold planning (Raibert + capture point) and min-jerk swing trajectories.

#ifndef HYPERDOG_LOCOMOTION__SWING_HPP_
#define HYPERDOG_LOCOMOTION__SWING_HPP_

#include "hyperdog_locomotion/math_utils.hpp"

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

struct FootholdParams
{
  double capture_point_gain{1.0};
  double centrifugal_gain{0.5};
  double max_step_offset{0.12};
  double touchdown_depth{0.0};
};

/// p_td = p_hip(t_td) + v T_st/2 + k_cp sqrt(h/g) (v - v_des) + k_c h/g (v x w_des)
Vec3 plan_foothold(
  const FootholdParams & prm, const Vec3 & nominal_body, const Vec3 & base_pos, double yaw,
  const Vec3 & v_world, const Vec3 & v_des_world, double yaw_rate_des, double t_remaining,
  double t_stance, double height);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__SWING_HPP_
