// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/planning/foothold_planner.hpp"

namespace hyperdog_locomotion
{

Vec3 plan_foothold(
  const FootholdParams & prm, const Vec3 & nominal_body, const Vec3 & base_pos, double yaw,
  const Vec3 & v_world, const Vec3 & v_des_world, double yaw_rate_des, double t_remaining,
  double t_stance, double height)
{
  const double yaw_td = yaw + yaw_rate_des * t_remaining;
  const Vec3 hip_td = base_pos + v_des_world * t_remaining + rot_z(yaw_td) * nominal_body;
  const double h = std::max(height, 0.05);
  Vec3 offset = v_world * t_stance * 0.5 +
    prm.capture_point_gain * std::sqrt(h / kGravity) * (v_world - v_des_world) +
    prm.centrifugal_gain * h / kGravity * v_world.cross(Vec3(0.0, 0.0, yaw_rate_des));
  offset.z() = 0.0;
  const double n = offset.head<2>().norm();
  if (n > prm.max_step_offset) {
    offset *= prm.max_step_offset / n;
  }
  Vec3 p = hip_td + offset;
  p.z() = 0.0;
  return p;
}

}  // namespace hyperdog_locomotion
