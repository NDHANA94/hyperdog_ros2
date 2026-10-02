// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/estimation/attitude_filter.hpp"

namespace hyperdog_locomotion
{

Eigen::Quaterniond MahonyFilter::update(const Vec3 & gyro, const Vec3 & accel, double dt)
{
  Vec3 e = Vec3::Zero();
  const double n = accel.norm();
  if (n > 0.5 * kGravity && n < 1.5 * kGravity) {
    const Vec3 v_meas = accel / n;
    const Vec3 v_est = q_.toRotationMatrix().transpose() * Vec3::UnitZ();
    e = v_meas.cross(v_est);
    bias_ -= ki_ * e * dt;
  }
  const Vec3 w = gyro - bias_ + kp_ * e;
  const Eigen::Quaterniond dq(0.0, w.x(), w.y(), w.z());
  const Eigen::Quaterniond qd = q_ * dq;
  q_.coeffs() += 0.5 * qd.coeffs() * dt;
  q_.normalize();
  return q_;
}

}  // namespace hyperdog_locomotion
