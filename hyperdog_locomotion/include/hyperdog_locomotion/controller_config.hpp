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
// Complete configuration of the locomotion controller. The structure mirrors
// the sections of config/locomotion.yaml; defaults equal the shipped file.

#ifndef HYPERDOG_LOCOMOTION__CONTROLLER_CONFIG_HPP_
#define HYPERDOG_LOCOMOTION__CONTROLLER_CONFIG_HPP_

#include <map>
#include <string>

#include "hyperdog_locomotion/common/math.hpp"
#include "hyperdog_locomotion/control/convex_mpc.hpp"
#include "hyperdog_locomotion/control/leg_controller.hpp"
#include "hyperdog_locomotion/control/qp_balance_controller.hpp"
#include "hyperdog_locomotion/estimation/contact_estimator.hpp"
#include "hyperdog_locomotion/estimation/kinematic_kalman_filter.hpp"
#include "hyperdog_locomotion/kinematics/leg_kinematics.hpp"
#include "hyperdog_locomotion/planning/disturbance_monitor.hpp"
#include "hyperdog_locomotion/planning/foothold_planner.hpp"
#include "hyperdog_locomotion/planning/gait_scheduler.hpp"

namespace hyperdog_locomotion
{

/// robot: rigid body model used by the force controllers
struct BodyModelParams
{
  double mass{7.4};                       // [kg]
  Vec3 inertia{0.06, 0.12, 0.14};         // [kg m^2] lumped body + hips
};

/// locomotion: commands, limits and posture
struct LocomotionParams
{
  double body_height{0.24};
  double body_height_min{0.15};
  double body_height_max{0.28};
  double step_height{0.06};
  Vec3 max_velocity{0.7, 0.3, 1.2};       // vx, vy, wz
  Vec3 max_acceleration{1.5, 1.0, 3.0};
  double idle_time_to_stand{0.8};         // [s]
  bool terrain_adaptation{true};
  double stand_up_time{1.5};              // [s]
  double sit_down_time{1.2};              // [s]
};

/// balance: ground reaction force control
struct BalanceParams
{
  std::string controller{"mpc"};          // "mpc" | "qp"
  ContactLimits limits;
  QPBalanceParams qp;
  MPCParams mpc;
};

/// joint_gains used outside of the stance / swing controller
struct PostureGains
{
  Vec3 stand_up_kp{60, 60, 60};
  Vec3 stand_up_kd{1.5, 1.5, 1.5};
  Vec3 passive_kd{1.0, 1.0, 1.0};
};

/// estimation
struct EstimationParams
{
  std::string attitude_source{"imu_orientation"};   // "imu_orientation" | "mahony"
  double mahony_kp{1.0};
  double mahony_ki{0.01};
  ContactParams contact;
  KalmanParams kalman;
  double ground_plane_filter{0.02};
};

/// safety
struct SafetyParams
{
  bool fall_protection{true};
  double fall_angle{1.0};                 // [rad]
  Vec3 max_joint_torque{20, 20, 20};      // [Nm] clip of feed-forward torques
};

struct ControllerConfig
{
  double control_rate{500.0};             // [Hz]
  RobotGeometry geometry;
  BodyModelParams body;
  LocomotionParams locomotion;
  std::map<std::string, Gait> gaits{default_gaits()};
  FootholdParams foothold;
  BalanceParams balance;
  DisturbanceRecoveryParams recovery;
  PostureGains posture;
  LegControllerParams legs;
  EstimationParams estimation;
  SafetyParams safety;
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROLLER_CONFIG_HPP_
