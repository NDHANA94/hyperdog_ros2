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
// PD control of the body pose produces a desired wrench which a QP distributes
// to the stance feet under friction pyramid constraints (used while standing,
// or always with balance.controller = "qp"). Forces act on the robot, world frame.

#ifndef HYPERDOG_LOCOMOTION__CONTROL__QP_BALANCE_CONTROLLER_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__QP_BALANCE_CONTROLLER_HPP_

#include "hyperdog_locomotion/control/force_control_common.hpp"
#include "hyperdog_locomotion/control/qp_solver.hpp"

namespace hyperdog_locomotion
{

struct QPBalanceParams
{
  Vec3 kp_position{60.0, 60.0, 300.0};
  Vec3 kd_position{14.0, 14.0, 30.0};
  Vec3 kp_orientation{300.0, 300.0, 120.0};
  Vec3 kd_orientation{30.0, 30.0, 18.0};
  Eigen::Matrix<double, 6,
    1> wrench_weights{(Eigen::Matrix<double, 6, 1>() << 1, 1, 5, 20, 20, 10).finished()};
  double force_regularization{1e-3};
  double force_smoothing{1e-3};
};

class QPBalanceController
{
public:
  QPBalanceController(
    const QPBalanceParams & p, const ContactLimits & lim, double mass,
    const Mat3 & inertia);
  Vec12 compute(
    const BodyState & s, const Mat3 & R_ref, const Vec3 & p_ref, const Vec3 & v_ref,
    const Vec3 & omega_ref, const Mat43 & feet_world, const Bool4 & contact, const Vec3 & normal);
  void reset();
  double solve_time() const {return solve_time_;}

private:
  QPBalanceParams p_;
  ContactLimits lim_;
  double mass_;
  Mat3 inertia_;
  QPSolver solver_;
  Vec12 f_prev_{Vec12::Zero()};
  double solve_time_{0.0};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__QP_BALANCE_CONTROLLER_HPP_
