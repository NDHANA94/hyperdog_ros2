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
// Single joint test through the BLDC impedance controller (bench / bring-up).
//
// The tested joint follows a sine (mode "sine") or a torque step (mode "step");
// all other joints hold the position they had at start with `hold_kp` / `hold_kd`.
// At the end a summary (tracking error, peak current, saturation, temperature)
// is printed and every joint is put into damping.
//
//   ros2 run hyperdog_bldc_control joint_test --ros-args -p joint:=FR_lleg_joint
//     -p mode:=sine -p amplitude:=0.2 -p frequency:=1.0 -p kp:=20.0 -p kd:=0.5

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hyperdog_msgs/msg/motor_commands.hpp"
#include "hyperdog_msgs/msg/motor_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointTest : public rclcpp::Node
{
public:
  JointTest()
  : Node("joint_test")
  {
    joint_ = declare_parameter("joint", std::string("FR_lleg_joint"));
    mode_ = declare_parameter("mode", std::string("sine"));
    amplitude_ = declare_parameter("amplitude", 0.2);      // [rad] sine amplitude
    frequency_ = declare_parameter("frequency", 1.0);      // [Hz]
    kp_ = declare_parameter("kp", 20.0);
    kd_ = declare_parameter("kd", 0.5);
    torque_ = declare_parameter("torque", 1.0);            // [Nm] step mode
    duration_ = declare_parameter("duration", 5.0);        // [s]
    hold_kp_ = declare_parameter("hold_kp", 20.0);
    hold_kd_ = declare_parameter("hold_kd", 0.5);
    max_amplitude_ = declare_parameter("max_amplitude", 0.6);  // safety limits
    max_kp_ = declare_parameter("max_kp", 60.0);
    max_torque_ = declare_parameter("max_torque", 8.0);
    if (std::abs(amplitude_) > max_amplitude_ || kp_ > max_kp_ || std::abs(torque_) > max_torque_) {
      throw std::invalid_argument("joint_test: amplitude / kp / torque above the safety limits");
    }

    cmd_pub_ = create_publisher<hyperdog_msgs::msg::MotorCommands>("bldc_controller/commands", 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        for (size_t k = 0; k < m->name.size() && k < m->position.size(); ++k) {
          q_[m->name[k]] = m->position[k];
        }
        have_js_ = true;
      });
    ms_sub_ = create_subscription<hyperdog_msgs::msg::MotorStates>(
      "bldc_controller/motor_states", rclcpp::SensorDataQoS(),
      [this](hyperdog_msgs::msg::MotorStates::ConstSharedPtr m) {
        for (size_t k = 0; k < m->name.size(); ++k) {
          if (m->name[k] != joint_ || !running_) {continue;}
          peak_current_ = std::max(peak_current_, std::abs(m->current[k]));
          max_temp_ = std::max(max_temp_, m->winding_temperature[k]);
          saturated_ += m->saturated[k] ? 1 : 0;
          ++state_samples_;
        }
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.002), [this]() {tick();});
  }

  bool done() const {return done_;}

private:
  void tick()
  {
    if (!have_js_ || q_.count(joint_) == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for joint_states ...");
      return;
    }
    const double now = get_clock()->now().seconds();
    if (!running_) {
      hold_ = q_;
      center_ = q_[joint_];
      t0_ = now;
      running_ = true;
      RCLCPP_INFO(
        get_logger(), "testing %s (%s) for %.1f s", joint_.c_str(), mode_.c_str(),
        duration_);
    }
    const double t = now - t0_;
    hyperdog_msgs::msg::MotorCommands c;
    c.header.stamp = get_clock()->now();
    for (const auto & [name, q0] : hold_) {
      c.name.push_back(name);
      if (t >= duration_) {
        // finished: damping on every joint
        c.position.push_back(q_[name]);
        c.velocity.push_back(0.0);
        c.effort.push_back(0.0);
        c.kp.push_back(0.0);
        c.kd.push_back(hold_kd_);
      } else if (name == joint_) {
        double q_des = center_, dq_des = 0.0, tau = 0.0, kp = kp_;
        if (mode_ == "sine") {
          const double w = 2.0 * M_PI * frequency_;
          q_des = center_ + amplitude_ * std::sin(w * t);
          dq_des = amplitude_ * w * std::cos(w * t);
          err2_ += std::pow(q_des - q_[name], 2);
          ++err_samples_;
        } else {   // step: pure torque after one second
          kp = 0.0;
          tau = t > 1.0 ? torque_ : 0.0;
          if (t <= 1.0) {step_q0_ = q_[name];}
          step_dq_ = q_[name] - step_q0_;
        }
        c.position.push_back(q_des);
        c.velocity.push_back(dq_des);
        c.effort.push_back(tau);
        c.kp.push_back(kp);
        c.kd.push_back(kd_);
      } else {
        c.position.push_back(q0);
        c.velocity.push_back(0.0);
        c.effort.push_back(0.0);
        c.kp.push_back(hold_kp_);
        c.kd.push_back(hold_kd_);
      }
    }
    cmd_pub_->publish(c);
    if (t >= duration_ + 0.5 && !done_) {
      done_ = true;
      RCLCPP_INFO(
        get_logger(),
        "\n  joint            %s\n  tracking RMS     %.4f rad\n  peak current     %.2f A\n"
        "  saturated        %.1f %% of samples\n  max winding temp %.1f C",
        joint_.c_str(), err_samples_ ? std::sqrt(err2_ / err_samples_) : 0.0, peak_current_,
        state_samples_ ? 100.0 * saturated_ / state_samples_ : 0.0, max_temp_);
      if (mode_ == "step") {
        RCLCPP_INFO(
          get_logger(), "  step displacement %+.4f rad (must have the sign of the torque %+.2f Nm)",
          step_dq_, torque_);
      }
    }
  }

  std::string joint_, mode_;
  double amplitude_, frequency_, kp_, kd_, torque_, duration_, hold_kp_, hold_kd_;
  double max_amplitude_, max_kp_, max_torque_;
  std::map<std::string, double> q_, hold_;
  bool have_js_{false}, running_{false}, done_{false};
  double center_{0.0}, t0_{0.0}, step_q0_{0.0}, step_dq_{0.0};
  double err2_{0.0}, peak_current_{0.0}, max_temp_{0.0};
  int err_samples_{0}, state_samples_{0}, saturated_{0};
  rclcpp::Publisher<hyperdog_msgs::msg::MotorCommands>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<hyperdog_msgs::msg::MotorStates>::SharedPtr ms_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTest>();
  while (rclcpp::ok() && !node->done()) {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
