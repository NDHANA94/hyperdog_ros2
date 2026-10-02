// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/estimator.hpp"

namespace hyperdog_locomotion
{

// ------------------------------------------------------------------ Mahony
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
  Eigen::Quaterniond dq(0.0, w.x(), w.y(), w.z());
  Eigen::Quaterniond qd = q_ * dq;
  q_.coeffs() += 0.5 * qd.coeffs() * dt;
  q_.normalize();
  return q_;
}

// ------------------------------------------------------------- Kalman filter
KinematicKalmanFilter::KinematicKalmanFilter(const KalmanParams & p, double dt)
: p_(p), dt_(dt)
{
  A_.setIdentity();
  A_.block<3, 3>(0, 3) = dt * Mat3::Identity();
  B_.setZero();
  B_.block<3, 3>(3, 0) = dt * Mat3::Identity();
  C_.setZero();
  for (int i = 0; i < 4; ++i) {
    C_.block<3, 3>(3 * i, 0) = Mat3::Identity();
    C_.block<3, 3>(3 * i, 6 + 3 * i) = -Mat3::Identity();
    C_.block<3, 3>(12 + 3 * i, 3) = Mat3::Identity();
    C_(24 + i, 8 + 3 * i) = 1.0;
  }
}

void KinematicKalmanFilter::reset(const Vec3 & base_pos, const Mat43 & feet_world)
{
  x_.setZero();
  x_.segment<3>(0) = base_pos;
  for (int i = 0; i < 4; ++i) {
    x_.segment<3>(6 + 3 * i) = feet_world.row(i).transpose();
  }
  P_ = Mat18::Identity() * 0.01;
}

void KinematicKalmanFilter::update(
  const Mat3 & R, const Vec3 & omega_body, const Vec3 & accel_body,
  const Mat43 & feet_body, const Mat43 & feet_vel_body, const std::array<double, 4> & trust,
  const std::array<double, 4> & foot_ground_z)
{
  const double dt = dt_;
  const Vec3 a_world = R * accel_body - Vec3(0.0, 0.0, kGravity);
  // predict
  x_ = A_ * x_ + B_ * a_world;
  Mat18 Q = Mat18::Zero();
  Q.block<3, 3>(0, 0) = Mat3::Identity() * p_.process_noise_position * dt;
  Q.block<3, 3>(3, 3) = Mat3::Identity() * p_.process_noise_velocity * dt * kGravity;
  for (int i = 0; i < 4; ++i) {
    const double scale = 1.0 + (1.0 - trust[i]) * p_.swing_noise_scale;
    Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = Mat3::Identity() * p_.process_noise_foot * dt * scale;
  }
  P_ = A_ * P_ * A_.transpose() + Q;

  // correct
  Eigen::Matrix<double, 28, 1> y;
  Eigen::Matrix<double, 28, 1> r_diag;
  const Vec3 omega_w = R * omega_body;
  for (int i = 0; i < 4; ++i) {
    const Vec3 p_rel = R * feet_body.row(i).transpose();
    const Vec3 v_rel = R * feet_vel_body.row(i).transpose() + omega_w.cross(p_rel);
    const double scale = 1.0 + (1.0 - trust[i]) * p_.swing_noise_scale;
    y.segment<3>(3 * i) = -p_rel;
    y.segment<3>(12 + 3 * i) = -v_rel;
    y(24 + i) = foot_ground_z[i];
    r_diag.segment<3>(3 * i).setConstant(p_.measurement_noise_position);
    r_diag.segment<3>(12 + 3 * i).setConstant(p_.measurement_noise_velocity * scale);
    r_diag(24 + i) = p_.measurement_noise_foot_height * scale;
  }
  const Eigen::Matrix<double, 28, 28> S = C_ * P_ * C_.transpose() +
    Eigen::Matrix<double, 28, 28>(r_diag.asDiagonal());
  const Eigen::Matrix<double, 18, 28> K =
    S.ldlt().solve(C_ * P_).transpose();
  x_ += K * (y - C_ * x_);
  P_ = (Mat18::Identity() - K * C_) * P_;
  P_ = 0.5 * (P_ + P_.transpose()).eval();
}

// ---------------------------------------------------------- contact estimator
void ContactEstimator::update(
  const Bool4 & scheduled, const std::array<double, 4> & progress, const Bool4 * sensor,
  const std::array<double, 4> * foot_fz)
{
  Bool4 measured = scheduled;
  if (p_.source == "sensor" && sensor) {
    measured = *sensor;
  } else if (p_.source == "torque" && foot_fz) {
    for (int i = 0; i < 4; ++i) {measured[i] = (*foot_fz)[i] > p_.force_threshold;}
  }
  for (int i = 0; i < 4; ++i) {
    early_[i] = false;
    late_[i] = false;
    if (scheduled[i]) {
      late_[i] = !measured[i] && progress[i] <= p_.late_contact_max_progress;
      contact_[i] = !late_[i];
    } else {
      early_[i] = measured[i] && progress[i] > p_.early_contact_min_progress;
      contact_[i] = early_[i];
    }
  }
}

std::array<double, 4> ContactEstimator::trust(
  const Bool4 & scheduled, const std::array<double, 4> & progress) const
{
  std::array<double, 4> t{0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    if (contact_[i] && scheduled[i]) {
      const double r = p_.stance_trust_ramp;
      t[i] = r > 0.0 ?
        std::max(0.05, std::min({1.0, progress[i] / r, (1.0 - progress[i]) / r + 0.2})) : 1.0;
    } else if (contact_[i]) {
      t[i] = 0.3;
    }
  }
  return t;
}

// ------------------------------------------------------------- ground plane
void GroundPlaneEstimator::reset(const Mat43 & feet_world, double foot_radius)
{
  hist_ = feet_world;
  const double z = feet_world.col(2).mean() - foot_radius;
  coef_ = Vec3(z, 0.0, 0.0);
}

void GroundPlaneEstimator::update(const Mat43 & feet_world, const Bool4 & use, double foot_radius)
{
  for (int i = 0; i < 4; ++i) {
    if (use[i]) {hist_.row(i) = feet_world.row(i);}
  }
  Eigen::Matrix<double, 4, 3> A;
  A.col(0).setOnes();
  A.col(1) = hist_.col(0);
  A.col(2) = hist_.col(1);
  const Eigen::Vector4d b = hist_.col(2).array() - foot_radius;
  const Vec3 c = A.colPivHouseholderQr().solve(b);
  if (c.allFinite()) {coef_ += alpha_ * (c - coef_);}
}

Eigen::Vector2d GroundPlaneEstimator::slope_rp(double yaw) const
{
  const double b = coef_[1], c = coef_[2];
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double sx = b * cy + c * sy;      // slope along heading
  const double sl = -b * sy + c * cy;     // slope sideways
  return Eigen::Vector2d(std::atan(sl), -std::atan(sx));
}

}  // namespace hyperdog_locomotion
