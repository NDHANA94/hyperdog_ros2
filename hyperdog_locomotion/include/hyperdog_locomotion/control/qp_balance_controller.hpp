// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
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
  Eigen::Matrix<double, 6, 1> wrench_weights{(Eigen::Matrix<double, 6, 1>() << 1, 1, 5, 20, 20, 10).finished()};
  double force_regularization{1e-3};
  double force_smoothing{1e-3};
};

class QPBalanceController
{
public:
  QPBalanceController(const QPBalanceParams & p, const ContactLimits & lim, double mass, const Mat3 & inertia);
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
