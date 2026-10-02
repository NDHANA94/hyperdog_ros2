// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/control/qp_balance_controller.hpp"

#include <chrono>

namespace hyperdog_locomotion
{

namespace
{
double elapsed(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

QPBalanceController::QPBalanceController(
  const QPBalanceParams & p, const ContactLimits & lim, double mass, const Mat3 & inertia)
: p_(p), lim_(lim), mass_(mass), inertia_(inertia)
{
  QPSettings s;
  s.max_iter = 200;
  solver_ = QPSolver(s);
}

void QPBalanceController::reset()
{
  solver_.reset();
  f_prev_.setZero();
}

Vec12 QPBalanceController::compute(
  const BodyState & s, const Mat3 & R_ref, const Vec3 & p_ref, const Vec3 & v_ref,
  const Vec3 & omega_ref, const Mat43 & feet_world, const Bool4 & contact, const Vec3 & normal)
{
  const auto t0 = std::chrono::steady_clock::now();
  const Vec3 acc = p_.kp_position.cwiseProduct(p_ref - s.p) +
    p_.kd_position.cwiseProduct(v_ref - s.v);
  const Vec3 F = mass_ * (acc + Vec3(0.0, 0.0, kGravity));
  const Vec3 e_R = so3_log(R_ref * s.R.transpose());
  const Mat3 I_w = s.R * inertia_ * s.R.transpose();
  const Vec3 tau = I_w * (p_.kp_orientation.cwiseProduct(e_R) +
    p_.kd_orientation.cwiseProduct(omega_ref - s.omega_world));
  Eigen::Matrix<double, 6, 1> b;
  b << F, tau;
  Eigen::Matrix<double, 6, 12> A = Eigen::Matrix<double, 6, 12>::Zero();
  for (int i = 0; i < 4; ++i) {
    A.block<3, 3>(0, 3 * i) = Mat3::Identity();
    A.block<3, 3>(3, 3 * i) = skew(feet_world.row(i).transpose() - s.p);
  }
  const Eigen::Matrix<double, 6, 6> S = p_.wrench_weights.asDiagonal();
  const double reg = p_.force_regularization + p_.force_smoothing;
  const Eigen::MatrixXd P = 2.0 * (A.transpose() * S * A + reg * Eigen::Matrix<double, 12,
    12>::Identity());
  const Eigen::VectorXd q = -2.0 * (A.transpose() * S * b + p_.force_smoothing * f_prev_);
  Eigen::MatrixXd C;
  Eigen::VectorXd lo, hi;
  friction_constraints(contact, lim_, normal, C, lo, hi);
  Vec12 f = solver_.solve(P, q, C, lo, hi);
  for (int i = 0; i < 4; ++i) {
    if (!contact[i]) {f.segment<3>(3 * i).setZero();}
  }
  f_prev_ = f;
  solve_time_ = elapsed(t0);
  return f;
}

}  // namespace hyperdog_locomotion
