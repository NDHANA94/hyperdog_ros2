// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Mahony complementary filter (gyro + accelerometer) for IMUs that do not
// provide a fused orientation (selected with estimation.attitude_source).

#ifndef HYPERDOG_LOCOMOTION__ESTIMATION__ATTITUDE_FILTER_HPP_
#define HYPERDOG_LOCOMOTION__ESTIMATION__ATTITUDE_FILTER_HPP_

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

class MahonyFilter
{
public:
  explicit MahonyFilter(double kp = 1.0, double ki = 0.01)
  : kp_(kp), ki_(ki) {}

  Eigen::Quaterniond update(const Vec3 & gyro, const Vec3 & accel, double dt);
  void reset(const Eigen::Quaterniond & q) {q_ = q; bias_.setZero();}

private:
  double kp_;
  double ki_;
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Vec3 bias_{Vec3::Zero()};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__ESTIMATION__ATTITUDE_FILTER_HPP_
