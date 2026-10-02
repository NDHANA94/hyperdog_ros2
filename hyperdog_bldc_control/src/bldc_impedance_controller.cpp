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

#include "hyperdog_bldc_control/bldc_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

// get_value()/set_value() changed signature across ros2_control releases;
// these helpers keep the controller source compatible with all of them.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
namespace
{
template<typename T>
double read_iface(const T & iface) {return iface.get_value();}
template<typename T>
void write_iface(T & iface, double v) {(void)iface.set_value(v);}
}  // namespace
#pragma GCC diagnostic pop

namespace hyperdog_bldc_control
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn BldcImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("joint_motor_types", std::vector<std::string>{});
    auto_declare<std::vector<double>>("joint_lower_limits", std::vector<double>{});
    auto_declare<std::vector<double>>("joint_upper_limits", std::vector<double>{});
    auto_declare<double>("limit_stiffness", 50.0);
    auto_declare<bool>("simulate_motor_dynamics", true);
    auto_declare<double>("command_timeout", 0.25);
    auto_declare<double>("timeout_kd", 1.0);
    auto_declare<double>("state_publish_rate", 100.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

BldcMotorParams BldcImpedanceController::load_motor(const std::string & type)
{
  auto node = get_node();
  BldcMotorParams p;
  p.name = type;
  auto get = [&](const std::string & key, double def) {
    const std::string name = "motors." + type + "." + key;
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, def);
    }
    return node->get_parameter(name).as_double();
  };
  p.kv_rpm_per_volt = get("kv_rpm_per_volt", p.kv_rpm_per_volt);
  p.torque_constant = get("torque_constant", p.torque_constant);
  p.phase_resistance = get("phase_resistance", p.phase_resistance);
  p.phase_inductance = get("phase_inductance", p.phase_inductance);
  p.pole_pairs = static_cast<int>(get("pole_pairs", p.pole_pairs));
  p.bus_voltage = get("bus_voltage", p.bus_voltage);
  p.voltage_utilization = get("voltage_utilization", p.voltage_utilization);
  p.max_current = get("max_current", p.max_current);
  p.rated_current = get("rated_current", p.rated_current);
  p.current_loop_bandwidth = get("current_loop_bandwidth", p.current_loop_bandwidth);
  p.gear_ratio = get("gear_ratio", p.gear_ratio);
  p.gearbox_efficiency = get("gearbox_efficiency", p.gearbox_efficiency);
  p.rotor_inertia = get("rotor_inertia", p.rotor_inertia);
  p.coulomb_friction = get("coulomb_friction", p.coulomb_friction);
  p.viscous_friction = get("viscous_friction", p.viscous_friction);
  p.max_velocity = get("max_velocity", p.max_velocity);
  p.thermal_resistance = get("thermal_resistance", p.thermal_resistance);
  p.thermal_capacitance = get("thermal_capacitance", p.thermal_capacitance);
  p.ambient_temperature = get("ambient_temperature", p.ambient_temperature);
  p.derate_start_temperature = get("derate_start_temperature", p.derate_start_temperature);
  p.max_winding_temperature = get("max_winding_temperature", p.max_winding_temperature);
  p.mass = get("mass", p.mass);
  return p;
}

CallbackReturn BldcImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "parameter 'joints' is empty");
    return CallbackReturn::ERROR;
  }
  auto types = node->get_parameter("joint_motor_types").as_string_array();
  if (types.size() == 1) {
    types.assign(joints_.size(), types[0]);
  }
  if (types.size() != joints_.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "'joint_motor_types' must have 1 or %zu entries", joints_.size());
    return CallbackReturn::ERROR;
  }
  motors_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto p = load_motor(types[i]);
    motors_.emplace_back(p);
    RCLCPP_INFO(
      node->get_logger(),
      "%s: motor '%s' Kt=%.3f Nm/A N=%.1f peak=%.1f Nm rated=%.1f Nm no-load=%.1f rad/s",
      joints_[i].c_str(), p.name.c_str(), p.kt(), p.gear_ratio, p.peak_torque(),
      p.rated_torque(), p.no_load_speed());
  }
  lower_limits_ = node->get_parameter("joint_lower_limits").as_double_array();
  upper_limits_ = node->get_parameter("joint_upper_limits").as_double_array();
  if (lower_limits_.size() != joints_.size() || upper_limits_.size() != joints_.size()) {
    lower_limits_.assign(joints_.size(), -std::numeric_limits<double>::infinity());
    upper_limits_.assign(joints_.size(), std::numeric_limits<double>::infinity());
  }
  limit_stiffness_ = node->get_parameter("limit_stiffness").as_double();
  simulate_dynamics_ = node->get_parameter("simulate_motor_dynamics").as_bool();
  command_timeout_ = node->get_parameter("command_timeout").as_double();
  timeout_kd_ = node->get_parameter("timeout_kd").as_double();
  const double rate = node->get_parameter("state_publish_rate").as_double();
  state_publish_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
  last_torque_cmd_.assign(joints_.size(), 0.0);

  command_sub_ = node->create_subscription<hyperdog_msgs::msg::MotorCommands>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    std::bind(&BldcImpedanceController::command_callback, this, std::placeholders::_1));
  state_pub_ = node->create_publisher<hyperdog_msgs::msg::MotorStates>(
    "~/motor_states", rclcpp::SystemDefaultsQoS());
  rt_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<hyperdog_msgs::msg::MotorStates>>(
    state_pub_);
  return CallbackReturn::SUCCESS;
}

void BldcImpedanceController::command_callback(
  const hyperdog_msgs::msg::MotorCommands::SharedPtr msg)
{
  const size_t n = msg->name.size();
  auto arr_ok = [n](const std::vector<double> & v) {return v.empty() || v.size() == n;};
  if (n == 0 || msg->position.size() != n || !arr_ok(msg->velocity) || !arr_ok(msg->effort) ||
    !arr_ok(msg->kp) || !arr_ok(msg->kd))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "ignoring malformed MotorCommands message");
    return;
  }
  auto frame = std::make_shared<CommandFrame>();
  // start from damping mode for joints not present in the message
  frame->joints.assign(joints_.size(), JointCommand{});
  std::vector<bool> seen(joints_.size(), false);
  for (size_t k = 0; k < n; ++k) {
    auto it = std::find(joints_.begin(), joints_.end(), msg->name[k]);
    if (it == joints_.end()) {continue;}
    const size_t i = static_cast<size_t>(it - joints_.begin());
    auto & c = frame->joints[i];
    c.position = msg->position[k];
    c.velocity = msg->velocity.empty() ? 0.0 : msg->velocity[k];
    c.effort = msg->effort.empty() ? 0.0 : msg->effort[k];
    c.kp = msg->kp.empty() ? 0.0 : msg->kp[k];
    c.kd = msg->kd.empty() ? timeout_kd_ : msg->kd[k];
    seen[i] = true;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (!seen[i]) {frame->joints[i].kd = timeout_kd_;}
  }
  frame->valid = true;
  command_buffer_.writeFromNonRT(frame);
}

InterfaceConfiguration BldcImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

InterfaceConfiguration BldcImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

CallbackReturn BldcImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joints_.size() ||
    state_interfaces_.size() != 2 * joints_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "unexpected number of claimed interfaces");
    return CallbackReturn::ERROR;
  }
  command_buffer_.writeFromNonRT(std::make_shared<CommandFrame>());
  for (auto & m : motors_) {
    m.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BldcImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {
    write_iface(c, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BldcImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto frame_ptr = command_buffer_.readFromRT();
  const std::shared_ptr<CommandFrame> frame = frame_ptr ? *frame_ptr : nullptr;
  const double dt = period.seconds();
  // command age is measured in controller time (works with sim time and wall time)
  if (frame != last_frame_) {
    last_frame_ = frame;
    command_age_ = 0.0;
  } else {
    command_age_ += dt;
  }
  bool timed_out = !frame || !frame->valid;
  if (!timed_out && command_timeout_ > 0.0) {
    timed_out = command_age_ > command_timeout_;
  }
  state_pub_age_ += dt;
  const bool publish = rt_state_pub_ &&
    (state_publish_period_ <= 0.0 || state_pub_age_ >= state_publish_period_);
  bool locked = publish && rt_state_pub_->trylock();
  if (locked) {
    auto & m = rt_state_pub_->msg_;
    const size_t n = joints_.size();
    m.header.stamp = time;
    m.name = joints_;
    m.position.resize(n); m.velocity.resize(n); m.torque.resize(n); m.torque_command.resize(n);
    m.current.resize(n); m.electrical_power.resize(n); m.winding_temperature.resize(n);
    m.saturated.resize(n);
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    const double q = read_iface(state_interfaces_[2 * i]);
    const double dq = read_iface(state_interfaces_[2 * i + 1]);
    double tau_req = 0.0;
    if (timed_out) {
      tau_req = -timeout_kd_ * dq;
    } else {
      const auto & c = frame->joints[i];
      tau_req = c.kp * (c.position - q) + c.kd * (c.velocity - dq) + c.effort;
    }
    // soft joint limits
    if (q < lower_limits_[i]) {tau_req += limit_stiffness_ * (lower_limits_[i] - q);}
    if (q > upper_limits_[i]) {tau_req += limit_stiffness_ * (upper_limits_[i] - q);}
    if (!std::isfinite(tau_req)) {tau_req = 0.0;}
    const auto out = motors_[i].update(tau_req, dq, dt, simulate_dynamics_);
    write_iface(command_interfaces_[i], out.torque);
    last_torque_cmd_[i] = tau_req;
    if (locked) {
      auto & m = rt_state_pub_->msg_;
      m.position[i] = q;
      m.velocity[i] = dq;
      m.torque[i] = out.torque;
      m.torque_command[i] = tau_req;
      m.current[i] = out.current;
      m.electrical_power[i] = out.electrical_power;
      m.winding_temperature[i] = out.temperature;
      m.saturated[i] = out.saturated;
    }
  }
  if (locked) {
    rt_state_pub_->unlockAndPublish();
    state_pub_age_ = 0.0;
  }
  return controller_interface::return_type::OK;
}

}  // namespace hyperdog_bldc_control

PLUGINLIB_EXPORT_CLASS(
  hyperdog_bldc_control::BldcImpedanceController, controller_interface::ControllerInterface)
