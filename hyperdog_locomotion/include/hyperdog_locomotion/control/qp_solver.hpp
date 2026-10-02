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
//
// Dense, dependency free QP solver (ADMM with the OSQP splitting):
//     minimize 0.5 x'Px + q'x   subject to   l <= A x <= u
// with diagonal pre-conditioning, over-relaxation and warm starting.
// Sized for the balance QP (12 variables) and the condensed MPC (12*N).

#ifndef HYPERDOG_LOCOMOTION__CONTROL__QP_SOLVER_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__QP_SOLVER_HPP_

#include <Eigen/Dense>

namespace hyperdog_locomotion
{

struct QPSettings
{
  double rho{0.1};
  double sigma{1e-6};
  double alpha{1.6};
  int max_iter{400};
  double eps_abs{1e-4};
  double eps_rel{1e-4};
};

class QPSolver
{
public:
  explicit QPSolver(const QPSettings & s = QPSettings())
  : s_(s) {}

  Eigen::VectorXd solve(
    const Eigen::MatrixXd & P, const Eigen::VectorXd & q, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & l, const Eigen::VectorXd & u);

  void reset() {x_.resize(0); z_.resize(0); y_.resize(0);}
  int iterations() const {return iterations_;}
  bool converged() const {return converged_;}
  QPSettings & settings() {return s_;}

private:
  QPSettings s_;
  Eigen::VectorXd x_, z_, y_;
  int iterations_{0};
  bool converged_{false};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__QP_SOLVER_HPP_
