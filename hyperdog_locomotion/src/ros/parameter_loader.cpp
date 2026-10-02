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

#include "parameter_loader.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace hyperdog_locomotion
{

namespace
{
class ParamReader
{
public:
  explicit ParamReader(rclcpp::Node & node)
  : node_(node) {}

  template<typename T>
  void get(const std::string & name, T & value)
  {
    value = node_.declare_parameter<T>(name, value);
  }

  void get(const std::string & name, int & value)
  {
    value = static_cast<int>(node_.declare_parameter<int64_t>(name, value));
  }

  void get(const std::string & name, Vec3 & value)
  {
    std::vector<double> v{value.x(), value.y(), value.z()};
    get_array(name, v, 3);
    value = Vec3(v[0], v[1], v[2]);
  }

  void get_array(const std::string & name, std::vector<double> & value, size_t size)
  {
    value = node_.declare_parameter<std::vector<double>>(name, value);
    if (value.size() != size) {
      throw std::invalid_argument(
              "parameter '" + name + "' must have " + std::to_string(size) + " elements");
    }
  }

private:
  rclcpp::Node & node_;
};
}  // namespace

ControllerConfig load_controller_config(rclcpp::Node & node)
{
  ControllerConfig c;
  ParamReader r(node);
  r.get("control_rate", c.control_rate);

  // robot
  auto & g = c.geometry;
  std::vector<double> hip{g.hip_x, g.hip_y};
  r.get_array("robot.hip_offset", hip, 2);
  g.hip_x = hip[0];
  g.hip_y = hip[1];
  r.get("robot.abad_length", g.abad_length);
  r.get("robot.upper_length", g.upper_length);
  r.get("robot.lower_length", g.lower_length);
  r.get("robot.foot_radius", g.foot_radius);
  r.get("robot.joint_lower_limits", g.q_min);
  r.get("robot.joint_upper_limits", g.q_max);
  r.get("robot.mass", c.body.mass);
  r.get("robot.body_inertia", c.body.inertia);

  // locomotion
  auto & l = c.locomotion;
  r.get("locomotion.body_height", l.body_height);
  std::vector<double> hl{l.body_height_min, l.body_height_max};
  r.get_array("locomotion.body_height_limits", hl, 2);
  l.body_height_min = hl[0];
  l.body_height_max = hl[1];
  r.get("locomotion.step_height", l.step_height);
  r.get("locomotion.max_velocity", l.max_velocity);
  r.get("locomotion.max_acceleration", l.max_acceleration);
  r.get("locomotion.idle_time_to_stand", l.idle_time_to_stand);
  r.get("locomotion.terrain_adaptation", l.terrain_adaptation);
  r.get("locomotion.stand_up_time", l.stand_up_time);
  r.get("locomotion.sit_down_time", l.sit_down_time);

  // gaits
  std::vector<std::string> names{"trot", "walk", "pace", "bound", "pronk"};
  r.get("gait_names", names);
  for (const auto & n : names) {
    Gait gt = c.gaits.count(n) ? c.gaits[n] : c.gaits["trot"];
    gt.name = n;
    r.get("gaits." + n + ".period", gt.period);
    r.get("gaits." + n + ".duty", gt.duty);
    std::vector<double> off(gt.offsets.begin(), gt.offsets.end());
    r.get_array("gaits." + n + ".offsets", off, 4);
    std::copy(off.begin(), off.end(), gt.offsets.begin());
    c.gaits[n] = gt;
  }

  // foothold planning and swing
  r.get("foothold.capture_point_gain", c.foothold.capture_point_gain);
  r.get("foothold.centrifugal_gain", c.foothold.centrifugal_gain);
  r.get("foothold.max_step_offset", c.foothold.max_step_offset);
  r.get("swing.touchdown_depth", c.legs.touchdown_depth);
  r.get("swing.late_contact_reach", c.legs.late_contact_reach);
  r.get("swing.max_joint_velocity", c.legs.max_joint_velocity);

  // balance
  auto & b = c.balance;
  r.get("balance.controller", b.controller);
  r.get("balance.friction_coefficient", b.limits.mu);
  r.get("balance.min_normal_force", b.limits.fz_min);
  r.get("balance.max_normal_force", b.limits.fz_max);
  r.get("balance.qp.kp_position", b.qp.kp_position);
  r.get("balance.qp.kd_position", b.qp.kd_position);
  r.get("balance.qp.kp_orientation", b.qp.kp_orientation);
  r.get("balance.qp.kd_orientation", b.qp.kd_orientation);
  std::vector<double> ww(b.qp.wrench_weights.data(), b.qp.wrench_weights.data() + 6);
  r.get_array("balance.qp.wrench_weights", ww, 6);
  for (int i = 0; i < 6; ++i) {
    b.qp.wrench_weights[i] = ww[i];
  }
  r.get("balance.qp.force_regularization", b.qp.force_regularization);
  r.get("balance.qp.force_smoothing", b.qp.force_smoothing);
  r.get("balance.mpc.horizon", b.mpc.horizon);
  r.get("balance.mpc.dt", b.mpc.dt);
  r.get("balance.mpc.update_period", b.mpc.update_period);
  r.get("balance.mpc.max_iterations", b.mpc.max_iterations);
  std::vector<double> sw(b.mpc.state_weights.begin(), b.mpc.state_weights.end());
  r.get_array("balance.mpc.state_weights", sw, 13);
  std::copy(sw.begin(), sw.end(), b.mpc.state_weights.begin());
  r.get("balance.mpc.force_weight", b.mpc.force_weight);

  // disturbance recovery
  auto & d = c.recovery;
  r.get("disturbance_recovery.enabled", d.enabled);
  r.get("disturbance_recovery.velocity_threshold", d.velocity_threshold);
  r.get("disturbance_recovery.tilt_threshold", d.tilt_threshold);
  r.get("disturbance_recovery.capture_point_margin", d.capture_point_margin);
  r.get("disturbance_recovery.settle_velocity", d.settle_velocity);
  r.get("disturbance_recovery.settle_time", d.settle_time);
  r.get("disturbance_recovery.gait", d.gait);

  // joint gains
  r.get("joint_gains.stand_up_kp", c.posture.stand_up_kp);
  r.get("joint_gains.stand_up_kd", c.posture.stand_up_kd);
  r.get("joint_gains.passive_kd", c.posture.passive_kd);
  r.get("joint_gains.stance_kp", c.legs.stance_kp);
  r.get("joint_gains.stance_kd", c.legs.stance_kd);
  r.get("joint_gains.swing_kp", c.legs.swing_kp);
  r.get("joint_gains.swing_kd", c.legs.swing_kd);
  r.get("joint_gains.swing_cartesian_kp", c.legs.swing_cartesian_kp);
  r.get("joint_gains.swing_cartesian_kd", c.legs.swing_cartesian_kd);
  r.get("joint_gains.leg_gravity_compensation_mass", c.legs.leg_gravity_compensation_mass);

  // estimation
  auto & e = c.estimation;
  r.get("estimation.attitude_source", e.attitude_source);
  r.get("estimation.mahony_kp", e.mahony_kp);
  r.get("estimation.mahony_ki", e.mahony_ki);
  r.get("estimation.contact_source", e.contact.source);
  r.get("estimation.contact_force_threshold", e.contact.force_threshold);
  r.get("estimation.early_contact_min_swing_progress", e.contact.early_contact_min_progress);
  r.get("estimation.late_contact_max_stance_progress", e.contact.late_contact_max_progress);
  r.get("estimation.stance_trust_ramp", e.contact.stance_trust_ramp);
  r.get("estimation.process_noise_position", e.kalman.process_noise_position);
  r.get("estimation.process_noise_velocity", e.kalman.process_noise_velocity);
  r.get("estimation.process_noise_foot", e.kalman.process_noise_foot);
  r.get("estimation.measurement_noise_position", e.kalman.measurement_noise_position);
  r.get("estimation.measurement_noise_velocity", e.kalman.measurement_noise_velocity);
  r.get("estimation.measurement_noise_foot_height", e.kalman.measurement_noise_foot_height);
  r.get("estimation.swing_noise_scale", e.kalman.swing_noise_scale);
  r.get("estimation.ground_plane_filter", e.ground_plane_filter);

  // safety
  r.get("safety.fall_protection", c.safety.fall_protection);
  r.get("safety.fall_angle", c.safety.fall_angle);
  r.get("safety.max_joint_torque", c.safety.max_joint_torque);
  return c;
}

}  // namespace hyperdog_locomotion
