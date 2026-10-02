// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/control/force_control_common.hpp"

#include <limits>

namespace hyperdog_locomotion
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

void friction_constraints(
  const Bool4 & contact, const ContactLimits & lim, const Vec3 & normal_in,
  Eigen::MatrixXd & A, Eigen::VectorXd & l, Eigen::VectorXd & u, int col_offset, int total_cols)
{
  const Vec3 n = normal_in.normalized();
  Vec3 t1 = Vec3::UnitY().cross(n).normalized();
  const Vec3 t2 = n.cross(t1);
  int rows = 0;
  for (int i = 0; i < 4; ++i) {
    rows += contact[i] ? 5 : 3;
  }
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

}  // namespace hyperdog_locomotion
