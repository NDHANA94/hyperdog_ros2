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

#include "hyperdog_locomotion/control/self_righting.hpp"

#include <algorithm>
#include <cmath>

namespace hyperdog_locomotion
{

namespace
{
Vec12 tile(const Vec3 & v)
{
  Vec12 o;
  for (int i = 0; i < 4; ++i) {
    o.segment<3>(3 * i) = v;
  }
  return o;
}

bool is_right_leg(int leg) {return leg == 0 || leg == 2;}   // FR, BR
}  // namespace

bool SelfRighting::lower_side(int leg) const
{
  return (ground_side_ > 0.0) == is_right_leg(leg);
}

double SelfRighting::tilt(const Vec3 & rpy)
{
  return std::acos(std::clamp(std::cos(rpy.x()) * std::cos(rpy.y()), -1.0, 1.0));
}

void SelfRighting::start(const Vec12 & q_now)
{
  phase_ = Phase::SETTLE;
  t_ = 0.0;
  rest_t_ = 0.0;
  attempts_ = 0;
  q_start_ = q_now;
}

SelfRighting::Phase SelfRighting::step(
  double dt, const Vec3 & rpy, const Vec3 & omega, const Vec12 & q_now,
  const Vec12 & tuck_pose, MotorCommand & out)
{
  t_ += dt;
  out.q = q_now;
  out.dq.setZero();
  out.tau.setZero();
  out.kp.setZero();
  out.kd = tile(p_.kd);

  switch (phase_) {
    case Phase::SETTLE:
      rest_t_ = omega.norm() < p_.rest_rate ? rest_t_ + dt : 0.0;
      if (t_ >= p_.settle_time && rest_t_ >= 0.3) {
        if (tilt(rpy) < p_.upright_angle) {
          phase_ = Phase::DONE;
        } else if (attempts_ >= p_.max_attempts) {
          phase_ = Phase::FAILED;
        } else {
          // world-up components of the body y and z axes (ZYX Euler angles)
          const double up_y = std::cos(rpy.y()) * std::sin(rpy.x());
          const double up_z = std::cos(rpy.y()) * std::cos(rpy.x());
          ground_side_ = up_y >= 0.0 ? 1.0 : -1.0;   // left side up: right side is lower
          on_back_ = up_z < -0.5;
          released_ = false;
          q_start_ = q_now;
          phase_ = Phase::TUCK;
          t_ = 0.0;
        }
      }
      break;
    case Phase::TUCK: {
        const double a = smoothstep_cos(t_ / p_.tuck_time);
        out.q = (1.0 - a) * q_start_ + a * tuck_pose;
        out.kp = tile(p_.kp);
        if (t_ >= p_.tuck_time) {
          q_roll_ = out.q;
          phase_ = Phase::ROLL;
          t_ = 0.0;
        }
        break;
      }
    case Phase::ROLL: {
        const double T = on_back_ ? p_.flip_time : p_.roll_time;
        const double a = smoothstep_cos(t_ / T);
        for (int i = 0; i < 4; ++i) {
          const Vec3 q0 = q_roll_.segment<3>(3 * i);
          Vec3 goal = q0;
          if (lower_side(i)) {
            goal = on_back_ ? p_.flip_push_pose : p_.push_pose;
          } else if (on_back_) {
            goal = p_.flip_pivot_pose;
          }
          out.q.segment<3>(3 * i) = (1.0 - a) * q0 + a * goal;
          out.kp.segment<3>(3 * i) = on_back_ && lower_side(i) ? p_.flip_kp : p_.kp;
        }
        if (on_back_ && !released_ && t_ > 0.5 * T && tilt(rpy) < p_.release_tilt) {
          released_ = true;
        }
        if (released_) {out.kp.setZero();}   // limp: tip over the edge
        if (t_ >= T + 0.3) {
          ++attempts_;
          phase_ = Phase::SETTLE;   // limp, then decide from the new orientation
          t_ = 0.0;
          rest_t_ = 0.0;
        }
        break;
      }
    case Phase::DONE:
    case Phase::FAILED:
      break;
  }
  return phase_;
}

}  // namespace hyperdog_locomotion
