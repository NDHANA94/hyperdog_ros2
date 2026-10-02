// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// ROS-agnostic closed loop locomotion controller of HyperDog.
//
// Every control tick:
//   sensors -> attitude -> contact estimation -> kinematic Kalman filter
//   -> gait scheduler (+ automatic stepping on disturbances)
//   -> foothold planner + swing trajectories
//   -> body controller (convex MPC while stepping, QP balance while standing)
//   -> stance torques tau = -J^T R^T f, swing cartesian impedance
//   -> per joint MIT-mode commands (q, dq, kp, kd, tau_ff) for the BLDC actuators
//
// FSM: PASSIVE -> STAND_UP -> BALANCE <-> LOCOMOTION, SIT -> PASSIVE,
// with fall protection (-> PASSIVE / damping).

#ifndef HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_
#define HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hyperdog_locomotion/balance.hpp"
#include "hyperdog_locomotion/estimator.hpp"
#include "hyperdog_locomotion/gait_scheduler.hpp"
#include "hyperdog_locomotion/kinematics.hpp"
#include "hyperdog_locomotion/swing.hpp"

namespace hyperdog_locomotion
{

enum class Mode {PASSIVE, STAND_UP, BALANCE, LOCOMOTION, SIT};
const char * to_string(Mode m);

struct ControllerConfig
{
  double control_rate{500.0};
  RobotGeometry geometry;
  double mass{7.4};
  Vec3 body_inertia{0.06, 0.12, 0.14};

  // locomotion
  double body_height{0.24};
  double body_height_min{0.15};
  double body_height_max{0.28};
  double step_height{0.06};
  Vec3 max_velocity{0.5, 0.3, 1.2};
  Vec3 max_acceleration{1.5, 1.0, 3.0};
  double idle_time_to_stand{0.8};
  bool terrain_adaptation{true};
  double stand_up_time{1.5};
  double sit_down_time{1.2};
  std::map<std::string, Gait> gaits{default_gaits()};
  FootholdParams foothold;

  // balance
  std::string balance_controller{"mpc"};   // mpc | qp
  ContactLimits contact_limits;
  QPBalanceParams qp;
  MPCParams mpc;

  // disturbance rejection
  bool recovery_enabled{true};
  double recovery_velocity_threshold{0.25};
  double recovery_tilt_threshold{0.2};
  double recovery_capture_margin{0.03};
  double recovery_settle_velocity{0.06};
  double recovery_settle_time{0.6};
  std::string recovery_gait{"trot"};

  // joint level gains (hip, uleg, lleg)
  Vec3 stand_up_kp{60, 60, 60}, stand_up_kd{1.5, 1.5, 1.5};
  Vec3 stance_kp{5, 5, 5}, stance_kd{0.5, 0.5, 0.5};
  Vec3 swing_kp{10, 10, 10}, swing_kd{0.8, 0.8, 0.8};
  Vec3 passive_kd{1.0, 1.0, 1.0};
  Vec3 swing_cartesian_kp{200, 200, 200};
  Vec3 swing_cartesian_kd{6, 6, 6};
  double leg_gravity_compensation_mass{0.6};

  // estimation
  std::string attitude_source{"imu_orientation"};   // imu_orientation | mahony
  double mahony_kp{1.0};
  double mahony_ki{0.01};
  ContactParams contact;
  KalmanParams kalman;
  double ground_plane_filter{0.02};

  // safety
  bool fall_protection{true};
  double fall_angle{1.0};
  Vec3 max_joint_torque{20, 20, 20};
};

struct SensorData
{
  Vec12 q{Vec12::Zero()};
  Vec12 dq{Vec12::Zero()};
  Vec12 tau{Vec12::Zero()};
  std::optional<Eigen::Quaterniond> imu_orientation;
  Vec3 gyro{Vec3::Zero()};                   // body frame
  Vec3 accel{0.0, 0.0, kGravity};            // specific force, body frame
  std::optional<Bool4> foot_contact;
};

struct Command
{
  Mode mode{Mode::PASSIVE};
  std::string gait{"trot"};
  double vx{0.0}, vy{0.0}, wz{0.0};
  double body_height{0.0};   // <= 0 keep
  double step_height{0.0};   // <= 0 keep
  Vec3 body_rpy{Vec3::Zero()};
};

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
  Mode mode;
  std::string gait;
  std::string controller;
  Bool4 contact;
  Bool4 scheduled_contact;
  std::array<double, 4> phase;
  Vec12 foot_force;
  Vec3 rpy;
  Vec3 position;
  Vec3 velocity;
  Vec3 omega;
  double height;
  bool recovering;
  double solve_time_ms;
  Mat43 feet_world;      // estimated foot positions (world)
  Mat43 foot_target;     // commanded foot positions (world)
};

class LocomotionController
{
public:
  explicit LocomotionController(const ControllerConfig & cfg);

  void set_command(const Command & c) {cmd_ = c;}
  const Command & command() const {return cmd_;}
  MotorCommand step(const SensorData & s);
  Diagnostics diagnostics() const;
  Mode mode() const {return mode_;}
  /// events since the last call (mode changes, disturbances, falls)
  std::vector<std::pair<double, std::string>> take_events();
  double dt() const {return dt_;}

private:
  void update_attitude(const SensorData & s);
  void estimate(const SensorData & s);
  std::array<double, 4> foot_forces_from_torque(const SensorData & s) const;
  void handle_mode(const SensorData & s);
  void set_mode(Mode m);
  Vec12 joint_pose(double height) const;
  void stand_up(const SensorData & s, MotorCommand & out);
  void sit(const SensorData & s, MotorCommand & out);
  Vec12 gravity_feedforward(const SensorData & s) const;
  void init_balance(const SensorData & s);
  Vec3 update_velocity_command();
  void select_gait(const Vec3 & target);
  Eigen::Vector2d terrain_rp() const;
  void locomotion(const SensorData & s, MotorCommand & out);

  ControllerConfig cfg_;
  double dt_;
  RobotKinematics kin_;
  Mat3 inertia_;
  QPBalanceController qp_;
  ConvexMPC mpc_;
  int mpc_decimation_{10};
  MahonyFilter mahony_;
  KinematicKalmanFilter kf_;
  ContactEstimator contacts_;
  GroundPlaneEstimator ground_;
  GaitScheduler gait_;

  Mode mode_{Mode::PASSIVE};
  double mode_time_{0.0};
  double time_{0.0};
  long tick_{0};
  Command cmd_;
  Vec3 v_des_{Vec3::Zero()};   // ramped body-frame vx, vy, wz
  Vec3 p_ref_{Vec3::Zero()};
  double yaw_ref_{0.0};
  double height_ref_;
  double step_height_;
  Vec12 f_des_{Vec12::Zero()};
  Mat3 R_{Mat3::Identity()};
  Vec3 rpy_{Vec3::Zero()};
  Vec3 omega_body_{Vec3::Zero()};
  Mat43 feet_body_{Mat43::Zero()};
  Mat43 feet_world_{Mat43::Zero()};
  Bool4 stance_{true, true, true, true};
  Mat43 liftoff_{Mat43::Zero()};
  Mat43 foothold_{Mat43::Zero()};
  Mat43 anchor_{Mat43::Zero()};
  Mat43 foot_target_{Mat43::Zero()};
  Bool4 prev_sched_{true, true, true, true};
  bool recovering_{false};
  double settle_timer_{0.0};
  double idle_timer_{0.0};
  Vec12 start_q_{Vec12::Zero()};
  std::string active_ctrl_{"qp"};
  double solve_time_{0.0};
  std::vector<std::pair<double, std::string>> events_;
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_
