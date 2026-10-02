// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Turns the body-level plan into per-joint MIT-mode commands
// (q, dq, kp, kd, tau_ff) for the BLDC actuators:
//   stance leg: tau_ff = -J^T R^T f (ground reaction force), soft impedance
//               around the touchdown point; a late touchdown keeps reaching down
//   swing leg:  min-jerk trajectory tracked by joint impedance + cartesian
//               impedance feed-forward + leg gravity / inertia compensation

#ifndef HYPERDOG_LOCOMOTION__CONTROL__LEG_CONTROLLER_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__LEG_CONTROLLER_HPP_

#include <array>

#include "hyperdog_locomotion/common/math.hpp"
#include "hyperdog_locomotion/controller_types.hpp"
#include "hyperdog_locomotion/kinematics/leg_kinematics.hpp"

namespace hyperdog_locomotion
{

struct LegControllerParams
{
  Vec3 stance_kp{5, 5, 5};
  Vec3 stance_kd{0.5, 0.5, 0.5};
  Vec3 swing_kp{10, 10, 10};
  Vec3 swing_kd{0.8, 0.8, 0.8};
  Vec3 swing_cartesian_kp{200, 200, 200};
  Vec3 swing_cartesian_kd{6, 6, 6};
  double leg_gravity_compensation_mass{0.6};  // [kg]
  double touchdown_depth{0.0};                // [m] swing ends this far below the ground estimate
  double late_contact_reach{0.02};            // [m] extra reach for a late touchdown
  double max_joint_velocity{20.0};            // [rad/s] clamp of the velocity reference
};

/// Body state and plan used by the leg controller for one tick.
struct LegControlInput
{
  Mat3 R{Mat3::Identity()};            // body orientation
  Vec3 p{Vec3::Zero()};                // base position (world)
  Vec3 v{Vec3::Zero()};                // base velocity (world)
  Vec3 omega_body{Vec3::Zero()};       // angular velocity (body)
  Vec12 q{Vec12::Zero()};
  Vec12 dq{Vec12::Zero()};
  Mat43 feet_body{Mat43::Zero()};      // current foot positions (body)
  Bool4 stance{true, true, true, true};
  Bool4 late{false, false, false, false};
  Mat43 anchor{Mat43::Zero()};         // touchdown points of stance feet (world)
  Mat43 liftoff{Mat43::Zero()};        // lift-off points of swing feet (world)
  Mat43 foothold{Mat43::Zero()};       // planned touchdown points (world)
  std::array<double, 4> progress{0, 0, 0, 0};  // swing progress [0, 1]
  double swing_time{0.2};
  double step_height{0.06};
  Vec12 forces{Vec12::Zero()};         // ground reaction forces (world, on the robot)
};

class LegController
{
public:
  LegController(const LegControllerParams & p, const RobotKinematics & kin)
  : p_(p), kin_(kin) {}

  /// Fills q, dq, kp, kd and tau of `out`; returns the commanded foot positions (world).
  Mat43 compute(const LegControlInput & in, MotorCommand & out) const;

private:
  LegControllerParams p_;
  const RobotKinematics & kin_;
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__LEG_CONTROLLER_HPP_
