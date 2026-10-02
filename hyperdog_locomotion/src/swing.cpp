// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/swing.hpp"

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
