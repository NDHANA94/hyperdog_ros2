// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/kinematics/leg_kinematics.hpp"

#include <complex>

namespace hyperdog_locomotion
{

std::vector<std::string> joint_names()
{
  std::vector<std::string> n;
  for (const char * leg : kLegNames) {
    n.push_back(std::string(leg) + "_hip_joint");
    n.push_back(std::string(leg) + "_uleg_joint");
    n.push_back(std::string(leg) + "_lleg_joint");
  }
  return n;
}

LegKinematics::LegKinematics(int index, const RobotGeometry & g)
{
  const std::string name = kLegNames[index];
  side_ = name[1] == 'R' ? -1.0 : 1.0;
  const double sx = name[0] == 'F' ? 1.0 : -1.0;
  hip_ = Vec3(sx * g.hip_x, side_ * g.hip_y, 0.0);
  l1_ = g.abad_length;
  l2_ = g.upper_length;
  l3_ = g.lower_length;
  q_min_ = g.q_min;
  q_max_ = g.q_max;
  axes_ = {Vec3(side_, 0.0, 0.0), Vec3(0.0, -1.0, 0.0), Vec3(0.0, 1.0, 0.0)};
}

void LegKinematics::frames(
  const Vec3 & q, std::array<Vec3, 3> & p, std::array<Vec3, 3> & axes, Vec3 & foot) const
{
  const Mat3 R0 = Eigen::AngleAxisd(q[0], axes_[0]).toRotationMatrix();
  const Mat3 R1 = R0 * Eigen::AngleAxisd(q[1], axes_[1]).toRotationMatrix();
  const Mat3 R2 = R1 * Eigen::AngleAxisd(q[2], axes_[2]).toRotationMatrix();
  p[0] = hip_;
  p[1] = p[0] + R0 * Vec3(0.0, side_ * l1_, 0.0);
  p[2] = p[1] + R1 * Vec3(-l2_, 0.0, 0.0);
  foot = p[2] + R2 * Vec3(l3_, 0.0, 0.0);
  axes = {axes_[0], R0 * axes_[1], R1 * axes_[2]};
}

Vec3 LegKinematics::forward(const Vec3 & q) const
{
  std::array<Vec3, 3> p, a;
  Vec3 f;
  frames(q, p, a, f);
  return f;
}

Mat3 LegKinematics::jacobian(const Vec3 & q) const
{
  std::array<Vec3, 3> p, a;
  Vec3 f;
  frames(q, p, a, f);
  Mat3 J;
  for (int i = 0; i < 3; ++i) {
    J.col(i) = a[i].cross(f - p[i]);
  }
  return J;
}

bool LegKinematics::inverse(const Vec3 & p_foot, Vec3 & q) const
{
  const Vec3 p = p_foot - hip_;
  bool reachable = true;
  // ab/ad: the leg plane is offset by side*l1 from the hip axis
  double r_yz2 = p.y() * p.y() + p.z() * p.z();
  if (r_yz2 < l1_ * l1_ + 1e-6) {
    r_yz2 = l1_ * l1_ + 1e-6;
    reachable = false;
  }
  double z_s = -std::sqrt(r_yz2 - l1_ * l1_);
  const double psi = wrap_angle(std::atan2(p.z(), p.y()) - std::atan2(z_s, side_ * l1_));
  double x_s = p.x();
  // sagittal two-link chain: foot = e^{i q1} (-l2 + l3 e^{-i q2})
  double d = std::hypot(x_s, z_s);
  const double d_min = std::abs(l2_ - l3_) + 1e-4;
  const double d_max = l2_ + l3_ - 1e-4;
  if (d > d_max || d < d_min) {
    reachable = false;
    const double dc = std::clamp(d, d_min, d_max);
    const double s = dc / std::max(d, 1e-9);
    x_s *= s;
    z_s *= s;
    d = dc;
  }
  const double cos_q2 = (l2_ * l2_ + l3_ * l3_ - d * d) / (2.0 * l2_ * l3_);
  const double q2 = std::acos(std::clamp(cos_q2, -1.0, 1.0));
  const std::complex<double> inner(-l2_ + l3_ * std::cos(q2), -l3_ * std::sin(q2));
  double q1 = wrap_angle(std::atan2(z_s, x_s) - std::arg(inner));
  if (q1 < q_min_[1] - 0.5) {q1 += 2.0 * M_PI;}
  q = Vec3(side_ * psi, q1, q2);
  for (int i = 0; i < 3; ++i) {
    if (q[i] < q_min_[i] - 1e-6 || q[i] > q_max_[i] + 1e-6) {reachable = false;}
    q[i] = std::clamp(q[i], q_min_[i], q_max_[i]);
  }
  return reachable;
}

Vec3 LegKinematics::nominal_foot(double height) const
{
  return hip_ + Vec3(0.0, side_ * l1_, -height);
}

RobotKinematics::RobotKinematics(const RobotGeometry & g)
: geom_(g)
{
  for (int i = 0; i < 4; ++i) {
    legs_[i] = LegKinematics(i, g);
  }
}

Mat43 RobotKinematics::forward_all(const Vec12 & q) const
{
  Mat43 f;
  for (int i = 0; i < 4; ++i) {
    f.row(i) = legs_[i].forward(q.segment<3>(3 * i)).transpose();
  }
  return f;
}

std::array<Mat3, 4> RobotKinematics::jacobians(const Vec12 & q) const
{
  std::array<Mat3, 4> J;
  for (int i = 0; i < 4; ++i) {
    J[i] = legs_[i].jacobian(q.segment<3>(3 * i));
  }
  return J;
}

bool RobotKinematics::inverse_all(const Mat43 & feet, Vec12 & q) const
{
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    Vec3 qi;
    ok = legs_[i].inverse(feet.row(i).transpose(), qi) && ok;
    q.segment<3>(3 * i) = qi;
  }
  return ok;
}

}  // namespace hyperdog_locomotion
