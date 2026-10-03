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

#include "hyperdog_locomotion/control/leg_controller.hpp"

#include "hyperdog_locomotion/planning/swing_trajectory.hpp"

namespace hyperdog_locomotion
{

Mat43 LegController::compute(const LegControlInput & in, MotorCommand & out) const
{
  Mat43 targets = Mat43::Zero();
  const auto J = kin_.jacobians(in.q);
  const Mat3 RT = in.R.transpose();
  const Vec3 g_comp = RT * Vec3(0.0, 0.0, p_.leg_gravity_compensation_mass * kGravity);
  for (int i = 0; i < 4; ++i) {
    Vec3 p_b, v_b, tau, kp, kd;
    if (in.stance[i]) {
      Vec3 target_w = in.anchor.row(i).transpose();
      if (in.late[i]) {
        // expected touchdown did not happen yet: keep reaching down
        target_w.z() -= p_.late_contact_reach;
        kp = p_.swing_kp;
        kd = p_.swing_kd;
        tau = J[i].transpose() * g_comp;
      } else {
        kp = p_.stance_kp;
        kd = p_.stance_kd;
        tau = J[i].transpose() * (RT * -Vec3(in.forces.segment<3>(3 * i)));
      }
      targets.row(i) = target_w.transpose();
      p_b = RT * (target_w - in.p);
      v_b = -RT * in.v - in.omega_body.cross(p_b);   // stance foot is static in the world
    } else {
      const SwingSample sw = swing_trajectory(
        in.liftoff.row(i).transpose(), in.foothold.row(i).transpose(), in.step_height,
        in.progress[i], in.swing_time, p_.touchdown_depth, in.swing_shape[i]);
      targets.row(i) = sw.pos.transpose();
      p_b = RT * (sw.pos - in.p);
      v_b = RT * (sw.vel - in.v) - in.omega_body.cross(p_b);
      kp = p_.swing_kp;
      kd = p_.swing_kd;
      const Vec3 v_foot = J[i] * in.dq.segment<3>(3 * i);
      Vec3 f_c = p_.swing_cartesian_kp.cwiseProduct(p_b - in.feet_body.row(i).transpose()) +
        p_.swing_cartesian_kd.cwiseProduct(v_b - v_foot);
      f_c += g_comp + p_.leg_gravity_compensation_mass * (RT * sw.acc);
      tau = J[i].transpose() * f_c;
    }
    Vec3 q_des;
    kin_.leg(i).inverse(p_b, q_des);
    Vec3 dq_des = J[i].fullPivLu().solve(v_b);
    if (!dq_des.allFinite()) {dq_des.setZero();}
    out.q.segment<3>(3 * i) = q_des;
    out.dq.segment<3>(
      3 *
      i) = dq_des.cwiseMax(-p_.max_joint_velocity).cwiseMin(p_.max_joint_velocity);
    out.kp.segment<3>(3 * i) = kp;
    out.kd.segment<3>(3 * i) = kd;
    out.tau.segment<3>(3 * i) = tau;
  }
  return targets;
}

}  // namespace hyperdog_locomotion
