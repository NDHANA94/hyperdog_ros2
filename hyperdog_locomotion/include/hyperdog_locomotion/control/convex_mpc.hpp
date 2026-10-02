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
// Single rigid body convex model predictive control (Di Carlo et al. 2018):
// ground reaction forces over a receding horizon using the gait contact
// schedule (used while stepping). Forces act on the robot, world frame.

#ifndef HYPERDOG_LOCOMOTION__CONTROL__CONVEX_MPC_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__CONVEX_MPC_HPP_

#include <array>
#include <vector>

#include "hyperdog_locomotion/control/force_control_common.hpp"
#include "hyperdog_locomotion/control/qp_solver.hpp"

namespace hyperdog_locomotion
{

struct MPCParams
{
  int horizon{10};
  double dt{0.03};
  double update_period{0.01};
  int max_iterations{200};
  std::array<double, 13> state_weights{10, 10, 2, 2, 2, 100, 0.2, 0.2, 0.3, 1, 1, 1, 0};
  double force_weight{1e-5};
};

class ConvexMPC
{
public:
  ConvexMPC(const MPCParams & p, const ContactLimits & lim, double mass, const Mat3 & inertia);
  /// ref: N x 12 desired [rpy, p, omega_world, v]; contacts: N contact flags (k = 0 is now)
  Vec12 compute(
    const BodyState & s, const Eigen::MatrixXd & ref, const Mat43 & feet_world,
    const std::vector<Bool4> & contacts, const Vec3 & normal);
  void reset();
  double solve_time() const {return solve_time_;}
  const MPCParams & params() const {return p_;}

private:
  MPCParams p_;
  ContactLimits lim_;
  double mass_;
  Mat3 inertia_;
  QPSolver solver_;
  double solve_time_{0.0};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__CONVEX_MPC_HPP_
