// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/balance.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include <chrono>
#include <limits>

namespace hyperdog_locomotion
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
double elapsed(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

void friction_constraints(
  const Bool4 & contact, const ContactLimits & lim, const Vec3 & normal_in,
  Eigen::MatrixXd & A, Eigen::VectorXd & l, Eigen::VectorXd & u, int col_offset, int total_cols)
{
  const Vec3 n = normal_in.normalized();
  Vec3 t1 = Vec3::UnitY().cross(n).normalized();
  const Vec3 t2 = n.cross(t1);
  int rows = 0;
  for (int i = 0; i < 4; ++i) {rows += contact[i] ? 5 : 3;}
  A = Eigen::MatrixXd::Zero(rows, total_cols);
  l.resize(rows);
  u.resize(rows);
  int r = 0;
  for (int i = 0; i < 4; ++i) {
    const int c = col_offset + 3 * i;
    if (contact[i]) {
      A.block<1, 3>(r, c) = n.transpose();
      l[r] = lim.fz_min; u[r] = lim.fz_max; ++r;
      const Vec3 rows_v[4] = {t1 - lim.mu * n, -t1 - lim.mu * n, t2 - lim.mu * n, -t2 - lim.mu * n};
      for (const auto & v : rows_v) {
        A.block<1, 3>(r, c) = v.transpose();
        l[r] = -kInf; u[r] = 0.0; ++r;
      }
    } else {
      for (int k = 0; k < 3; ++k) {
        A(r, c + k) = 1.0;
        l[r] = 0.0; u[r] = 0.0; ++r;
      }
    }
  }
}

// =============================================================== QP balance
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
  const Vec3 acc = p_.kp_position.cwiseProduct(p_ref - s.p) + p_.kd_position.cwiseProduct(v_ref - s.v);
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
  const Eigen::MatrixXd P = 2.0 * (A.transpose() * S * A + reg * Eigen::Matrix<double, 12, 12>::Identity());
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

// =============================================================== convex MPC
ConvexMPC::ConvexMPC(const MPCParams & p, const ContactLimits & lim, double mass, const Mat3 & inertia)
: p_(p), lim_(lim), mass_(mass), inertia_(inertia)
{
  QPSettings s;
  s.max_iter = p.max_iterations;
  s.eps_abs = 1e-4;
  s.eps_rel = 1e-4;
  s.rho = 0.003;
  solver_ = QPSolver(s);
}

void ConvexMPC::reset() {solver_.reset();}

Vec12 ConvexMPC::compute(
  const BodyState & s, const Eigen::MatrixXd & ref, const Mat43 & feet_world,
  const std::vector<Bool4> & contacts, const Vec3 & normal)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int N = p_.horizon;
  const double dt = p_.dt;
  constexpr int nx = 13, nu = 12;
  Eigen::Matrix<double, nx, 1> x0;
  x0 << s.rpy, s.p, s.omega_world, s.v, -kGravity;
  const Mat3 Rz = rot_z(s.rpy.z());
  const Mat3 I_w = Rz * inertia_ * Rz.transpose();
  const Mat3 I_inv = I_w.inverse();

  // continuous dynamics, then exact discretisation via the matrix exponential
  Eigen::Matrix<double, nx + nu, nx + nu> M = Eigen::Matrix<double, nx + nu, nx + nu>::Zero();
  M.block<3, 3>(0, 6) = Rz.transpose();
  M.block<3, 3>(3, 9) = Mat3::Identity();
  M(11, 12) = 1.0;
  for (int i = 0; i < 4; ++i) {
    const Vec3 r = feet_world.row(i).transpose() - s.p;
    M.block<3, 3>(6, nx + 3 * i) = I_inv * skew(r);
    M.block<3, 3>(9, nx + 3 * i) = Mat3::Identity() / mass_;
  }
  const Eigen::Matrix<double, nx + nu, nx + nu> E = (M * dt).exp();
  const Eigen::Matrix<double, nx, nx> Ad = E.topLeftCorner<nx, nx>();
  const Eigen::Matrix<double, nx, nu> Bd = E.topRightCorner<nx, nu>();

  std::vector<Eigen::Matrix<double, nx, nx>> Apow(N + 1);
  Apow[0].setIdentity();
  for (int k = 1; k <= N; ++k) {Apow[k] = Ad * Apow[k - 1];}
  std::vector<Eigen::Matrix<double, nx, nu>> AB(N);
  for (int k = 0; k < N; ++k) {AB[k] = Apow[k] * Bd;}

  Eigen::MatrixXd Aqp(nx * N, nx);
  Eigen::MatrixXd Bqp = Eigen::MatrixXd::Zero(nx * N, nu * N);
  Eigen::VectorXd Xref(nx * N), Ldiag(nx * N);
  for (int k = 0; k < N; ++k) {
    Aqp.block<nx, nx>(nx * k, 0) = Apow[k + 1];
    for (int j = 0; j <= k; ++j) {
      Bqp.block<nx, nu>(nx * k, nu * j) = AB[k - j];
    }
    Xref.segment<12>(nx * k) = ref.row(std::min<int>(k, static_cast<int>(ref.rows()) - 1)).transpose();
    Xref(nx * k + 12) = -kGravity;
    for (int i = 0; i < nx; ++i) {Ldiag(nx * k + i) = p_.state_weights[i];}
  }
  const Eigen::MatrixXd LB = Ldiag.asDiagonal() * Bqp;
  Eigen::MatrixXd H = 2.0 * (Bqp.transpose() * LB);
  H.diagonal().array() += 2.0 * p_.force_weight;
  const Eigen::VectorXd g = 2.0 * LB.transpose() * (Aqp * x0 - Xref);

  // constraints
  std::vector<Eigen::MatrixXd> Cs(N);
  std::vector<Eigen::VectorXd> ls(N), us(N);
  int rows = 0;
  for (int k = 0; k < N; ++k) {
    const Bool4 & c = contacts[std::min<size_t>(k, contacts.size() - 1)];
    friction_constraints(c, lim_, normal, Cs[k], ls[k], us[k], nu * k, nu * N);
    rows += static_cast<int>(Cs[k].rows());
  }
  Eigen::MatrixXd C(rows, nu * N);
  Eigen::VectorXd lo(rows), hi(rows);
  int r = 0;
  for (int k = 0; k < N; ++k) {
    const int n = static_cast<int>(Cs[k].rows());
    C.middleRows(r, n) = Cs[k];
    lo.segment(r, n) = ls[k];
    hi.segment(r, n) = us[k];
    r += n;
  }
  const Eigen::VectorXd U = solver_.solve(H, g, C, lo, hi);
  Vec12 f = U.head<12>();
  for (int i = 0; i < 4; ++i) {
    if (!contacts[0][i]) {f.segment<3>(3 * i).setZero();}
  }
  solve_time_ = elapsed(t0);
  return f;
}

}  // namespace hyperdog_locomotion
