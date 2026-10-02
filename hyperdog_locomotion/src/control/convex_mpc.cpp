// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/control/convex_mpc.hpp"

#include <unsupported/Eigen/MatrixFunctions>

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

ConvexMPC::ConvexMPC(
  const MPCParams & p, const ContactLimits & lim, double mass,
  const Mat3 & inertia)
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
  for (int k = 1; k <= N; ++k) {
    Apow[k] = Ad * Apow[k - 1];
  }
  std::vector<Eigen::Matrix<double, nx, nu>> AB(N);
  for (int k = 0; k < N; ++k) {
    AB[k] = Apow[k] * Bd;
  }

  Eigen::MatrixXd Aqp(nx * N, nx);
  Eigen::MatrixXd Bqp = Eigen::MatrixXd::Zero(nx * N, nu * N);
  Eigen::VectorXd Xref(nx * N), Ldiag(nx * N);
  for (int k = 0; k < N; ++k) {
    Aqp.block<nx, nx>(nx * k, 0) = Apow[k + 1];
    for (int j = 0; j <= k; ++j) {
      Bqp.block<nx, nu>(nx * k, nu * j) = AB[k - j];
    }
    Xref.segment<12>(nx * k) = ref.row(
      std::min<int>(
        k,
        static_cast<int>(ref.rows()) - 1)).transpose();
    Xref(nx * k + 12) = -kGravity;
    for (int i = 0; i < nx; ++i) {
      Ldiag(nx * k + i) = p_.state_weights[i];
    }
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
