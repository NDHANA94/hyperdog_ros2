// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Ground reaction force controllers (forces act on the robot, world frame):
//  * QPBalanceController - PD on body pose -> desired wrench -> QP force
//    distribution with friction pyramids (used while standing).
//  * ConvexMPC - single rigid body convex MPC (Di Carlo et al. 2018) over a
//    receding horizon using the gait contact schedule (used while stepping).

#ifndef HYPERDOG_LOCOMOTION__BALANCE_HPP_
#define HYPERDOG_LOCOMOTION__BALANCE_HPP_

#include <array>
#include <vector>

#include "hyperdog_locomotion/math_utils.hpp"
#include "hyperdog_locomotion/qp_solver.hpp"

namespace hyperdog_locomotion
{

struct BodyState
{
  Mat3 R{Mat3::Identity()};
  Vec3 rpy{Vec3::Zero()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
  Vec3 omega_world{Vec3::Zero()};
};

struct ContactLimits
{
  double mu{0.6};
  double fz_min{3.0};
  double fz_max{160.0};
};

/// Friction pyramid rows for 4 feet (12 forces). Swing feet are constrained to zero.
void friction_constraints(
  const Bool4 & contact, const ContactLimits & lim, const Vec3 & normal,
  Eigen::MatrixXd & A, Eigen::VectorXd & l, Eigen::VectorXd & u, int col_offset = 0,
  int total_cols = 12);

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

#endif  // HYPERDOG_LOCOMOTION__BALANCE_HPP_
