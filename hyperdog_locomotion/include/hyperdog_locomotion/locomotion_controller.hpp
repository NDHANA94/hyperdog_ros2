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
// ROS-agnostic closed loop locomotion controller of HyperDog.
//
// Every control tick:
//   sensors -> attitude -> contact estimation -> kinematic Kalman filter
//   -> gait selection (+ automatic stepping on disturbances) -> gait scheduler
//   -> body reference -> foothold planner
//   -> ground reaction forces (convex MPC while stepping, QP while standing)
//   -> LegController: per joint MIT-mode commands for the BLDC actuators
//
// Finite state machine: PASSIVE -> STAND_UP -> BALANCE <-> LOCOMOTION,
// SIT -> PASSIVE, with fall protection: SELF_RIGHT (roll back over, then STAND_UP)
// or PASSIVE (damping) when self-righting is disabled or fails.

#ifndef HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_
#define HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "hyperdog_locomotion/control/convex_mpc.hpp"
#include "hyperdog_locomotion/control/leg_controller.hpp"
#include "hyperdog_locomotion/control/qp_balance_controller.hpp"
#include "hyperdog_locomotion/control/self_righting.hpp"
#include "hyperdog_locomotion/controller_config.hpp"
#include "hyperdog_locomotion/controller_types.hpp"
#include "hyperdog_locomotion/estimation/attitude_filter.hpp"
#include "hyperdog_locomotion/estimation/contact_estimator.hpp"
#include "hyperdog_locomotion/estimation/ground_plane_estimator.hpp"
#include "hyperdog_locomotion/estimation/kinematic_kalman_filter.hpp"
#include "hyperdog_locomotion/kinematics/leg_kinematics.hpp"
#include "hyperdog_locomotion/planning/disturbance_monitor.hpp"
#include "hyperdog_locomotion/planning/gait_scheduler.hpp"

namespace hyperdog_locomotion
{

class LocomotionController
{
public:
  explicit LocomotionController(const ControllerConfig & cfg);

  void set_command(const Command & c) {cmd_ = c;}
  const Command & command() const {return cmd_;}
  /// Advance the controller by one tick (1 / control_rate seconds).
  MotorCommand step(const SensorData & s);
  Diagnostics diagnostics() const;
  Mode mode() const {return mode_;}
  /// Events since the last call (mode changes, disturbances, falls) as (time, text).
  std::vector<std::pair<double, std::string>> take_events();
  double dt() const {return dt_;}

private:
  // --- estimation
  void update_attitude(const SensorData & s);
  void update_contacts(const SensorData & s);
  void estimate_state(const SensorData & s);
  std::array<double, 4> foot_forces_from_torque(const SensorData & s) const;
  Eigen::Vector2d terrain_rp() const;

  // --- finite state machine and posture motions
  void handle_mode(const SensorData & s);
  void set_mode(Mode m);
  Vec12 joint_pose(double height) const;
  Vec12 gravity_feedforward(const SensorData & s) const;
  void stand_up(const SensorData & s, MotorCommand & out);
  void sit(const SensorData & s, MotorCommand & out);
  void self_right(const SensorData & s, MotorCommand & out);
  void init_balance(const SensorData & s);

  // --- locomotion pipeline
  void locomotion(const SensorData & s, MotorCommand & out);
  Vec3 update_velocity_command();
  void select_gait(const Vec3 & target);
  Bool4 update_leg_phases();
  Vec3 update_body_reference();
  void update_footholds(const Bool4 & stance);
  void compute_ground_forces(const Bool4 & stance, const Vec3 & rpy_ref);

  ControllerConfig cfg_;
  double dt_;
  RobotKinematics kin_;
  Mat3 inertia_;
  QPBalanceController qp_;
  ConvexMPC mpc_;
  int mpc_decimation_{1};
  LegController legs_;
  MahonyFilter mahony_;
  KinematicKalmanFilter kf_;
  ContactEstimator contacts_;
  GroundPlaneEstimator ground_;
  GaitScheduler gait_;
  DisturbanceMonitor disturbance_;
  SelfRighting righting_;

  // finite state machine
  Mode mode_{Mode::PASSIVE};
  double mode_time_{0.0};
  double time_{0.0};
  int64_t tick_{0};
  Command cmd_;
  Vec12 start_q_{Vec12::Zero()};
  std::vector<std::pair<double, std::string>> events_;

  // estimated state
  Mat3 R_{Mat3::Identity()};
  Vec3 rpy_{Vec3::Zero()};
  Vec3 omega_body_{Vec3::Zero()};
  Mat43 feet_body_{Mat43::Zero()};
  Mat43 feet_world_{Mat43::Zero()};

  // references
  Vec3 v_des_{Vec3::Zero()};         // ramped body frame vx, vy, wz
  Vec3 v_des_world_{Vec3::Zero()};   // ramped velocity in the world frame (z = 0)
  Vec3 p_ref_{Vec3::Zero()};
  double yaw_ref_{0.0};
  double height_ref_;
  double step_height_;
  double idle_timer_{0.0};

  // per leg plan
  Bool4 stance_{true, true, true, true};
  Bool4 prev_sched_{true, true, true, true};
  Mat43 liftoff_{Mat43::Zero()};
  Mat43 foothold_{Mat43::Zero()};
  Mat43 anchor_{Mat43::Zero()};
  Mat43 foot_target_{Mat43::Zero()};

  // force control
  Vec12 f_des_{Vec12::Zero()};
  std::string active_ctrl_{"qp"};
  double solve_time_{0.0};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__LOCOMOTION_CONTROLLER_HPP_
