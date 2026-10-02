// Copyright 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hyperdog_locomotion/control/qp_solver.hpp"

#include <algorithm>
#include <cmath>

namespace hyperdog_locomotion
{

Eigen::VectorXd QPSolver::solve(
  const Eigen::MatrixXd & P_in, const Eigen::VectorXd & q_in, const Eigen::MatrixXd & A_in,
  const Eigen::VectorXd & l_in, const Eigen::VectorXd & u_in)
{
  const int n = static_cast<int>(P_in.rows());
  const int m = static_cast<int>(A_in.rows());

  // --- diagonal scaling: x = D x_s, rows of A normalised, cost normalised
  Eigen::VectorXd d = P_in.diagonal().cwiseAbs().cwiseMax(1e-12).cwiseSqrt().cwiseInverse();
  {
    Eigen::VectorXd tmp = d;
    std::nth_element(tmp.data(), tmp.data() + n / 2, tmp.data() + n);
    d /= tmp[n / 2];
  }
  Eigen::MatrixXd P = d.asDiagonal() * P_in * d.asDiagonal();
  Eigen::VectorXd q = d.cwiseProduct(q_in);
  Eigen::MatrixXd A = A_in * d.asDiagonal();
  Eigen::VectorXd e = A.rowwise().norm().cwiseMax(1e-9).cwiseInverse();
  A = e.asDiagonal() * A;
  Eigen::VectorXd l = l_in, u = u_in;
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(l[i])) {l[i] *= e[i];}
    if (std::isfinite(u[i])) {u[i] *= e[i];}
  }
  const double c = 1.0 / std::max(P.diagonal().cwiseAbs().mean(), 1e-12);
  P *= c;
  q *= c;

  // --- ADMM
  Eigen::VectorXd rho(m);
  for (int i = 0; i < m; ++i) {
    rho[i] = std::abs(u[i] - l[i]) < 1e-9 ? 1e3 * s_.rho : s_.rho;
  }
  Eigen::VectorXd x = (x_.size() == n) ? x_ : Eigen::VectorXd::Zero(n);
  Eigen::VectorXd z = (z_.size() == m) ? z_ : Eigen::VectorXd((A * x).cwiseMax(l).cwiseMin(u));
  Eigen::VectorXd y = (y_.size() == m) ? y_ : Eigen::VectorXd::Zero(m);

  Eigen::MatrixXd K = P + s_.sigma * Eigen::MatrixXd::Identity(n, n) +
    A.transpose() * rho.asDiagonal() * A;
  Eigen::LLT<Eigen::MatrixXd> llt(K);
  const Eigen::MatrixXd K_inv = llt.solve(Eigen::MatrixXd::Identity(n, n));
  const Eigen::MatrixXd At = A.transpose();

  converged_ = false;
  int it = 0;
  Eigen::VectorXd rhs(n), x_t(n), z_t(m), z_relax(m), z_new(m);
  for (it = 1; it <= s_.max_iter; ++it) {
    rhs.noalias() = s_.sigma * x - q + At * (rho.cwiseProduct(z) - y);
    x_t.noalias() = K_inv * rhs;
    z_t.noalias() = A * x_t;
    x = s_.alpha * x_t + (1.0 - s_.alpha) * x;
    z_relax = s_.alpha * z_t + (1.0 - s_.alpha) * z;
    z_new = (z_relax + y.cwiseQuotient(rho)).cwiseMax(l).cwiseMin(u);
    y += rho.cwiseProduct(z_relax - z_new);
    z = z_new;
    if (it % 10 == 0) {
      const Eigen::VectorXd Ax = A * x;
      const Eigen::VectorXd Px = P * x;
      const Eigen::VectorXd ATy = At * y;
      const double r_prim = m ? (Ax - z).cwiseAbs().maxCoeff() : 0.0;
      const double r_dual = (Px + q + ATy).cwiseAbs().maxCoeff();
      const double e_prim = s_.eps_abs + s_.eps_rel *
        (m ? std::max(Ax.cwiseAbs().maxCoeff(), z.cwiseAbs().maxCoeff()) : 0.0);
      const double e_dual = s_.eps_abs + s_.eps_rel *
        std::max({Px.cwiseAbs().maxCoeff(), ATy.cwiseAbs().maxCoeff(), q.cwiseAbs().maxCoeff()});
      if (r_prim < e_prim && r_dual < e_dual) {
        converged_ = true;
        break;
      }
    }
  }
  iterations_ = std::min(it, s_.max_iter);
  x_ = x;
  z_ = z;
  y_ = y;
  if (!x.allFinite()) {
    reset();
    return Eigen::VectorXd::Zero(n);
  }
  return d.cwiseProduct(x);
}

}  // namespace hyperdog_locomotion
