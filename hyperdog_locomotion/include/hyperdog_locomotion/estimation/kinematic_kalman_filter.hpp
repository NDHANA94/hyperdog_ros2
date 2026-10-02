// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
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
