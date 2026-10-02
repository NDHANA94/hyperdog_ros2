// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
#include <gtest/gtest.h>

#include "hyperdog_locomotion/control/convex_mpc.hpp"
#include "hyperdog_locomotion/control/leg_controller.hpp"
#include "hyperdog_locomotion/control/qp_balance_controller.hpp"

using hyperdog_locomotion::BodyState;
using hyperdog_locomotion::Bool4;
using hyperdog_locomotion::ContactLimits;
using hyperdog_locomotion::ConvexMPC;
using hyperdog_locomotion::LegControlInput;
using hyperdog_locomotion::LegController;
using hyperdog_locomotion::LegControllerParams;
using hyperdog_locomotion::MPCParams;
using hyperdog_locomotion::Mat3;
using hyperdog_locomotion::Mat43;
using hyperdog_locomotion::MotorCommand;
using hyperdog_locomotion::QPBalanceController;
using hyperdog_locomotion::QPBalanceParams;
using hyperdog_locomotion::QPSolver;
using hyperdog_locomotion::RobotKinematics;
using hyperdog_locomotion::Vec12;
using hyperdog_locomotion::Vec3;
using hyperdog_locomotion::kGravity;

namespace
{
Mat43 feet()
{
  Mat43 f;
  f << 0.175, -0.17, 0, 0.175, 0.17, 0, -0.175, -0.17, 0, -0.175, 0.17, 0;
  return f;
}
}  // namespace

TEST(QPSolver, BoxConstrainedQuadratic)
{
  // min (x-3)^2 + (y+1)^2  s.t. 0 <= x <= 1, y free  -> x = 1, y = -1
  QPSolver s;
  Eigen::MatrixXd P = 2.0 * Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd q(2);
  q << -6.0, 2.0;
  Eigen::MatrixXd A(1, 2);
  A << 1.0, 0.0;
  Eigen::VectorXd l(1), u(1);
  l << 0.0;
  u << 1.0;
  const auto x = s.solve(P, q, A, l, u);
  EXPECT_NEAR(x[0], 1.0, 1e-3);
  EXPECT_NEAR(x[1], -1.0, 1e-3);
}

TEST(QPBalance, StaticStandDistributesWeight)
{
  QPBalanceController c(QPBalanceParams(), ContactLimits(), 7.4,
    Vec3(0.06, 0.12, 0.14).asDiagonal());
  BodyState s;
  s.p = Vec3(0, 0, 0.24);
  const Vec12 f = c.compute(
    s, Mat3::Identity(), s.p, Vec3::Zero(), Vec3::Zero(), feet(),
    Bool4{true, true, true, true}, Vec3::UnitZ());
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(f[3 * i + 2], 7.4 * kGravity / 4.0, 0.2);
    EXPECT_NEAR(f[3 * i], 0.0, 0.2);
  }
}

TEST(QPBalance, RespectsFrictionAndSwingLegs)
{
  QPBalanceController c(QPBalanceParams(), ContactLimits(), 7.4,
    Vec3(0.06, 0.12, 0.14).asDiagonal());
  BodyState s;
  s.p = Vec3(0, 0, 0.24);
  s.v = Vec3(0.5, 0, 0);   // pushed forward
  const Vec12 f = c.compute(
    s, Mat3::Identity(), Vec3(0, 0, 0.24), Vec3::Zero(), Vec3::Zero(), feet(),
    Bool4{true, false, false, true}, Vec3::UnitZ());
  EXPECT_NEAR(f.segment<3>(3).norm(), 0.0, 1e-9);
  EXPECT_NEAR(f.segment<3>(6).norm(), 0.0, 1e-9);
  for (int i : {0, 3}) {
    EXPECT_LT(f[3 * i], 0.0);   // braking force
    EXPECT_LE(std::hypot(f[3 * i], f[3 * i + 1]), 0.6 * std::sqrt(2.0) * f[3 * i + 2] + 0.5);
  }
}

TEST(ConvexMPC, AcceleratesTowardsVelocityReference)
{
  ConvexMPC mpc(MPCParams(), ContactLimits(), 7.4, Vec3(0.06, 0.12, 0.14).asDiagonal());
  BodyState s;
  s.p = Vec3(0, 0, 0.24);
  const int N = 10;
  Eigen::MatrixXd ref = Eigen::MatrixXd::Zero(N, 12);
  for (int k = 0; k < N; ++k) {
    ref(k, 3) = 0.5 * 0.03 * k;
    ref(k, 5) = 0.24;
    ref(k, 9) = 0.5;
  }
  std::vector<Bool4> table(N, Bool4{true, false, false, true});
  for (int k = 5; k < N; ++k) {
    table[k] = Bool4{false, true, true, false};
  }
  const Vec12 f = mpc.compute(s, ref, feet(), table, Vec3::UnitZ());
  double fz = 0, fx = 0;
  for (int i = 0; i < 4; ++i) {
    fz += f[3 * i + 2]; fx += f[3 * i];
  }
  EXPECT_GT(fx, 5.0);
  EXPECT_NEAR(fz, 7.4 * kGravity, 25.0);
  EXPECT_NEAR(f.segment<3>(3).norm(), 0.0, 1e-9);
  EXPECT_LT(mpc.solve_time(), 0.05);
}

TEST(LegController, StanceTorqueSupportsWeight)
{
  RobotKinematics kin;
  LegController legs(LegControllerParams(), kin);
  LegControlInput in;
  in.p = Vec3(0, 0, 0.24);
  Mat43 feet_b;
  for (int i = 0; i < 4; ++i) {
    feet_b.row(i) = kin.leg(i).nominal_foot(0.22).transpose();
  }
  kin.inverse_all(feet_b, in.q);
  in.feet_body = feet_b;
  for (int i = 0; i < 4; ++i) {
    in.anchor.row(i) = (in.p + feet_b.row(i).transpose()).transpose();
    in.forces.segment<3>(3 * i) = Vec3(0, 0, 18.0);
  }
  MotorCommand out;
  const Mat43 targets = legs.compute(in, out);
  const auto J = kin.jacobians(in.q);
  for (int i = 0; i < 4; ++i) {
    // tau = -J^T f: the foot pushes down with the commanded force
    const Vec3 f_foot = J[i].transpose().fullPivLu().solve(Vec3(out.tau.segment<3>(3 * i)));
    EXPECT_NEAR(f_foot.z(), -18.0, 1e-6);
    // impedance set point is the current configuration (foot on its anchor)
    EXPECT_LT((out.q.segment<3>(3 * i) - in.q.segment<3>(3 * i)).norm(), 1e-6);
    EXPECT_LT((targets.row(i) - in.anchor.row(i)).norm(), 1e-9);
  }
}
