// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// State estimation:
//  * MahonyFilter            - attitude from gyro + accelerometer (real IMUs)
//  * KinematicKalmanFilter   - linear KF on [p, v, p_foot x4] fusing the IMU
//                              acceleration with leg kinematics of stance feet
//  * ContactEstimator        - fuses gait schedule with contact sensors or
//                              torque-based foot force estimates
//  * GroundPlaneEstimator    - least squares plane through the stance feet

#ifndef HYPERDOG_LOCOMOTION__ESTIMATOR_HPP_
#define HYPERDOG_LOCOMOTION__ESTIMATOR_HPP_

#include <array>
#include <string>

#include "hyperdog_locomotion/math_utils.hpp"

namespace hyperdog_locomotion
{

class MahonyFilter
{
public:
  MahonyFilter(double kp = 1.0, double ki = 0.01)
  : kp_(kp), ki_(ki) {}
  Eigen::Quaterniond update(const Vec3 & gyro, const Vec3 & accel, double dt);
  void reset(const Eigen::Quaterniond & q) {q_ = q; bias_.setZero();}

private:
  double kp_, ki_;
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Vec3 bias_{Vec3::Zero()};
};

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

struct ContactParams
{
  std::string source{"sensor"};          // sensor | torque | schedule
  double force_threshold{12.0};          // [N] for source = torque
  double early_contact_min_progress{0.6};
  double late_contact_max_progress{0.4};
  double stance_trust_ramp{0.15};
};

class ContactEstimator
{
public:
  explicit ContactEstimator(const ContactParams & p = ContactParams())
  : p_(p) {}
  /// measured: sensor contacts or nullptr; foot_fz: estimated normal forces or nullptr
  void update(
    const Bool4 & scheduled, const std::array<double, 4> & progress, const Bool4 * sensor,
    const std::array<double, 4> * foot_fz);
  std::array<double, 4> trust(const Bool4 & scheduled, const std::array<double, 4> & progress) const;

  const Bool4 & contact() const {return contact_;}
  const Bool4 & early() const {return early_;}
  const Bool4 & late() const {return late_;}
  const ContactParams & params() const {return p_;}

private:
  ContactParams p_;
  Bool4 contact_{true, true, true, true};
  Bool4 early_{false, false, false, false};
  Bool4 late_{false, false, false, false};
};

class GroundPlaneEstimator
{
public:
  explicit GroundPlaneEstimator(double alpha = 0.02)
  : alpha_(alpha) {}
  void reset(const Mat43 & feet_world, double foot_radius);
  void update(const Mat43 & feet_world, const Bool4 & use, double foot_radius);
  double height(double x, double y) const {return coef_[0] + coef_[1] * x + coef_[2] * y;}
  Vec3 normal() const {return Vec3(-coef_[1], -coef_[2], 1.0).normalized();}
  /// body roll / pitch that align the body with the plane for a heading `yaw`
  Eigen::Vector2d slope_rp(double yaw) const;

private:
  double alpha_;
  Vec3 coef_{Vec3::Zero()};
  Mat43 hist_{Mat43::Zero()};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__ESTIMATOR_HPP_
