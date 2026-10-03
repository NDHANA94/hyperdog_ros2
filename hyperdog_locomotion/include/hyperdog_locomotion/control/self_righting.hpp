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
// Self-righting after a fall. Joint space sequence:
//   SETTLE: damping until the body is at rest; then, from the resting orientation:
//           upright -> DONE, otherwise TUCK
//   TUCK:   fold all legs
//   ROLL:   lying on a side: the lower side legs swing over the body (thigh up to its
//           limit, knee extended) and push against the ground behind the body's top
//           surface, rolling it onto its belly.
//           Lying on the back: fast, stiff kick of the lower side legs while the other
//           legs fold inwards; once the body has turned past release_tilt all joints go
//           limp so that it tips onto its side (or belly).
//           Then SETTLE again (one attempt).
//           Limitation: with the +-1 rad hip range, the kick tips HyperDog onto its side but
//           it usually comes to rest leaning back (~110 deg tilt), from where the side roll
//           does not succeed; after max_attempts it stays in damping mode.
//   DONE:   body tilt (angle of the body z axis to vertical) below upright_angle
// After max_attempts attempts without success: FAILED.

#ifndef HYPERDOG_LOCOMOTION__CONTROL__SELF_RIGHTING_HPP_
#define HYPERDOG_LOCOMOTION__CONTROL__SELF_RIGHTING_HPP_

#include "hyperdog_locomotion/common/math.hpp"
#include "hyperdog_locomotion/controller_types.hpp"

namespace hyperdog_locomotion
{

struct SelfRightingParams
{
  bool enabled{true};
  int max_attempts{5};
  double settle_time{1.0};         // [s] damping before the first attempt
  double tuck_time{0.8};           // [s]
  double roll_time{1.2};           // [s]
  double upright_angle{0.3};       // [rad] body tilt below this: upright
  double rest_rate{0.5};           // [rad/s] body rate below which the body is at rest
  Vec3 push_pose{0.0, -1.2, 2.3};  // lower side leg joints at the end of ROLL
  double flip_time{0.1};           // [s] kick duration when lying on the back (needs momentum)
  Vec3 flip_kp{150, 150, 150};     // joint stiffness of the kick
  Vec3 flip_push_pose{1.0, -1.2, 2.3};  // kicking leg joints (hip inwards) when on the back
  Vec3 flip_pivot_pose{-1.0, 2.0, 2.3};  // the other legs during the kick
  double release_tilt{1.3};        // [rad] body tilt at which the kick releases
  Vec3 kp{40, 40, 40};             // joint gains during the motion (hip, uleg, lleg)
  Vec3 kd{1.0, 1.0, 1.0};
};

class SelfRighting
{
public:
  enum class Phase {SETTLE, TUCK, ROLL, DONE, FAILED};

  explicit SelfRighting(const SelfRightingParams & p = SelfRightingParams())
  : p_(p) {}

  void start(const Vec12 & q_now);
  /// rpy: body attitude, omega: body rate, tuck_pose: folded joint configuration.
  Phase step(
    double dt, const Vec3 & rpy, const Vec3 & omega, const Vec12 & q_now,
    const Vec12 & tuck_pose, MotorCommand & out);
  /// Angle between the body z axis and the vertical.
  static double tilt(const Vec3 & rpy);
  Phase phase() const {return phase_;}
  int attempts() const {return attempts_;}
  const SelfRightingParams & params() const {return p_;}

private:
  bool lower_side(int leg) const;

  SelfRightingParams p_;
  Phase phase_{Phase::SETTLE};
  double t_{0.0};
  double rest_t_{0.0};
  int attempts_{0};
  double ground_side_{1.0};   // +1: the right side is the lower one
  bool on_back_{false};
  bool released_{false};
  Vec12 q_roll_{Vec12::Zero()};   // joint command at the start of ROLL
  Vec12 q_start_{Vec12::Zero()};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__CONTROL__SELF_RIGHTING_HPP_
