// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Inputs and outputs of the locomotion controller (ROS independent).
// Leg order everywhere: FR, FL, BR, BL. Joint order per leg: hip, uleg, lleg.

#ifndef HYPERDOG_LOCOMOTION__CONTROLLER_TYPES_HPP_
#define HYPERDOG_LOCOMOTION__CONTROLLER_TYPES_HPP_

#include <array>
#include <optional>
#include <string>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

enum class Mode {PASSIVE, STAND_UP, BALANCE, LOCOMOTION, SIT};
const char * to_string(Mode m);

/// Measurements for one control tick.
struct SensorData
{
  Vec12 q{Vec12::Zero()};
  Vec12 dq{Vec12::Zero()};
  Vec12 tau{Vec12::Zero()};
  std::optional<Eigen::Quaterniond> imu_orientation;   // if the IMU provides one
  Vec3 gyro{Vec3::Zero()};                             // body frame [rad/s]
  Vec3 accel{0.0, 0.0, kGravity};                      // specific force, body frame [m/s^2]
  std::optional<Bool4> foot_contact;                   // if contact sensors are available
};

/// Operator / navigation command.
struct Command
{
  Mode mode{Mode::PASSIVE};
  std::string gait{"trot"};
  double vx{0.0}, vy{0.0}, wz{0.0};   // body frame
  double body_height{0.0};            // <= 0: keep current
  double step_height{0.0};            // <= 0: keep current
  Vec3 body_rpy{Vec3::Zero()};        // body attitude offset while standing
};

/// MIT-mode command for the 12 BLDC actuators.
struct MotorCommand
{
  Vec12 q{Vec12::Zero()};
  Vec12 dq{Vec12::Zero()};
  Vec12 tau{Vec12::Zero()};
  Vec12 kp{Vec12::Zero()};
  Vec12 kd{Vec12::Zero()};
};

struct Diagnostics
{
  Mode mode{Mode::PASSIVE};
  std::string gait;
  std::string controller;
  Bool4 contact{};
  Bool4 scheduled_contact{};
  std::array<double, 4> phase{};
  Vec12 foot_force{Vec12::Zero()};
  Vec3 rpy{Vec3::Zero()};
  Vec3 position{Vec3::Zero()};
  Vec3 velocity{Vec3::Zero()};
  Vec3 omega{Vec3::Zero()};
  double height{0.0};
  bool recovering{false};
  double solve_time_ms{0.0};
  Mat43 feet_world{Mat43::Zero()};    // estimated foot positions (world)
  Mat43 foot_target{Mat43::Zero()};   // commanded foot positions (world)
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROLLER_TYPES_HPP_
