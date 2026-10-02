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
// Linear Kalman filter on [p_base, v_base, p_foot x4] (world frame). The IMU
// acceleration drives the prediction; leg kinematics of feet in stable contact
// (assumed static) and their ground height correct it.

#ifndef HYPERDOG_LOCOMOTION__ESTIMATION__KINEMATIC_KALMAN_FILTER_HPP_
#define HYPERDOG_LOCOMOTION__ESTIMATION__KINEMATIC_KALMAN_FILTER_HPP_

#include <array>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct KalmanParams
{
  double process_noise_position{0.002};
  double process_noise_velocity{0.05};
  double process_noise_foot{0.002};
  double measurement_noise_position{0.002};
  double measurement_noise_velocity{0.05};
  double measurement_noise_foot_height{0.01};
  double swing_noise_scale{1e4};
  // [m/s^2] clamp of the gravity compensated IMU acceleration used for prediction;
  // rejects foot impact spikes (<= 0 disables)
  double max_acceleration{20.0};
  // [m/s] legs whose kinematic velocity differs more than this from the prediction are
  // ignored for the update (slip / impact rejection; <= 0 disables)
  double velocity_innovation_gate{0.5};
};

class KinematicKalmanFilter
{
public:
  using Vec18 = Eigen::Matrix<double, 18, 1>;
  using Mat18 = Eigen::Matrix<double, 18, 18>;

  KinematicKalmanFilter(const KalmanParams & p, double dt);
  void reset(const Vec3 & base_pos, const Mat43 & feet_world);
  /// trust[i] in [0, 1] = confidence that foot i is a static stance foot.
  void update(
    const Mat3 & R, const Vec3 & omega_body, const Vec3 & accel_body,
    const Mat43 & feet_body, const Mat43 & feet_vel_body, const std::array<double, 4> & trust,
    const std::array<double, 4> & foot_ground_z);

  Vec3 position() const {return x_.segment<3>(0);}
  Vec3 velocity() const {return x_.segment<3>(3);}
  Vec3 foot(int i) const {return x_.segment<3>(6 + 3 * i);}

private:
  KalmanParams p_;
  double dt_;
  Vec18 x_{Vec18::Zero()};
  Mat18 P_{Mat18::Identity() * 0.1};
  Mat18 A_;
  Eigen::Matrix<double, 18, 3> B_;
  Eigen::Matrix<double, 28, 18> C_;
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__ESTIMATION__KINEMATIC_KALMAN_FILTER_HPP_
