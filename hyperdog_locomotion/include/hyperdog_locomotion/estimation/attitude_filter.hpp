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
