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
// Gamepad teleoperation of HyperDog.
//   joy (sensor_msgs/Joy) -> cmd_vel (geometry_msgs/Twist)
//                         -> hyperdog/command (hyperdog_msgs/LocomotionCommand)
// Every axis / button index and scale is a parameter (config/joy_xbox.yaml).
//
// Default (Xbox style) mapping:
//   START   stand up + walk  /  sit down (toggle)      BACK  passive (damping, e-stop)
//   A trot   B walk   X pace   Y bound
//   left stick: vx / vy        right stick (horizontal): yaw rate
//   LB held + left stick: body roll / pitch while standing
//   d-pad up/down: body height (LB held: step height)
//   RB: dead-man switch (only if require_deadman is true)

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hyperdog_msgs/msg/locomotion_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using hyperdog_msgs::msg::LocomotionCommand;

class HyperdogTeleopJoy : public rclcpp::Node
{
public:
  HyperdogTeleopJoy()
  : Node("hyperdog_teleop_joy")
  {
    axis_vx_ = declare_parameter("axis.vx", 1);
    axis_vy_ = declare_parameter("axis.vy", 0);
    axis_wz_ = declare_parameter("axis.wz", 3);
    axis_height_ = declare_parameter("axis.height", 7);
    btn_start_ = declare_parameter("button.start", 7);
    btn_passive_ = declare_parameter("button.passive", 6);
    btn_body_ = declare_parameter("button.body_pose", 4);
    btn_deadman_ = declare_parameter("button.deadman", 5);
    gait_buttons_ = declare_parameter("button.gaits", std::vector<int64_t>{0, 1, 2, 3});
    gait_names_ = declare_parameter(
      "gaits",
      std::vector<std::string>{"trot", "walk", "pace", "bound"});
    max_vx_ = declare_parameter("scale.vx", 0.5);
    max_vy_ = declare_parameter("scale.vy", 0.3);
    max_wz_ = declare_parameter("scale.wz", 1.2);
    max_roll_ = declare_parameter("scale.roll", 0.3);
    max_pitch_ = declare_parameter("scale.pitch", 0.3);
    height_ = declare_parameter("body_height", 0.24);
    height_min_ = declare_parameter("body_height_min", 0.15);
    height_max_ = declare_parameter("body_height_max", 0.28);
    step_height_ = declare_parameter("step_height", 0.06);
    require_deadman_ = declare_parameter("require_deadman", false);
    deadzone_ = declare_parameter("deadzone", 0.08);

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cmd_pub_ = create_publisher<LocomotionCommand>("hyperdog/command", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {on_joy(*m);});
    cmd_.mode = LocomotionCommand::MODE_PASSIVE;
    cmd_.gait = gait_names_.empty() ? "trot" : gait_names_[0];
    RCLCPP_INFO(get_logger(), "HyperDog joystick teleop ready (START: stand/sit, BACK: passive)");
  }

private:
  double axis(const sensor_msgs::msg::Joy & j, int64_t i) const
  {
    if (i < 0 || static_cast<size_t>(i) >= j.axes.size()) {return 0.0;}
    const double v = j.axes[i];
    return std::abs(v) < deadzone_ ? 0.0 : v;
  }
  bool button(const sensor_msgs::msg::Joy & j, int64_t i) const
  {
    return i >= 0 && static_cast<size_t>(i) < j.buttons.size() && j.buttons[i];
  }
  bool pressed(const sensor_msgs::msg::Joy & j, int64_t i)
  {
    const bool now = button(j, i);
    const bool was = prev_buttons_.size() > static_cast<size_t>(std::max<int64_t>(
        i,
        0)) && i >= 0 &&
      prev_buttons_[i];
    return now && !was;
  }

  void on_joy(const sensor_msgs::msg::Joy & j)
  {
    bool changed = false;
    if (pressed(j, btn_passive_)) {
      cmd_.mode = LocomotionCommand::MODE_PASSIVE;
      changed = true;
    } else if (pressed(j, btn_start_)) {
      cmd_.mode = cmd_.mode == LocomotionCommand::MODE_LOCOMOTION ?
        LocomotionCommand::MODE_SIT : LocomotionCommand::MODE_LOCOMOTION;
      changed = true;
    }
    for (size_t k = 0; k < gait_buttons_.size() && k < gait_names_.size(); ++k) {
      if (pressed(j, gait_buttons_[k])) {
        cmd_.gait = gait_names_[k];
        changed = true;
      }
    }
    const bool body_mode = button(j, btn_body_);
    const double dpad = axis(j, axis_height_);
    if (dpad != 0.0 && (now() - last_height_change_).seconds() > 0.15) {
      if (body_mode) {
        step_height_ = std::clamp(step_height_ + 0.01 * (dpad > 0 ? 1 : -1), 0.02, 0.12);
      } else {
        height_ = std::clamp(height_ + 0.01 * (dpad > 0 ? 1 : -1), height_min_, height_max_);
      }
      last_height_change_ = now();
      changed = true;
    }
    geometry_msgs::msg::Vector3 rpy;
    if (body_mode) {
      rpy.x = -axis(j, axis_vy_) * max_roll_;
      rpy.y = axis(j, axis_vx_) * max_pitch_;
    }
    if (rpy.x != cmd_.body_rpy.x || rpy.y != cmd_.body_rpy.y) {
      cmd_.body_rpy = rpy;
      changed = true;
    }
    cmd_.body_height = height_;
    cmd_.step_height = step_height_;
    if (changed) {cmd_pub_->publish(cmd_);}

    geometry_msgs::msg::Twist t;
    const bool enabled = !require_deadman_ || button(j, btn_deadman_);
    if (enabled && !body_mode) {
      t.linear.x = axis(j, axis_vx_) * max_vx_;
      t.linear.y = axis(j, axis_vy_) * max_vy_;
      t.angular.z = axis(j, axis_wz_) * max_wz_;
    }
    twist_pub_->publish(t);
    prev_buttons_ = j.buttons;
  }

  int64_t axis_vx_, axis_vy_, axis_wz_, axis_height_;
  int64_t btn_start_, btn_passive_, btn_body_, btn_deadman_;
  std::vector<int64_t> gait_buttons_;
  std::vector<std::string> gait_names_;
  double max_vx_, max_vy_, max_wz_, max_roll_, max_pitch_;
  double height_, height_min_, height_max_, step_height_;
  bool require_deadman_;
  double deadzone_;
  LocomotionCommand cmd_;
  std::vector<int32_t> prev_buttons_;
  rclcpp::Time last_height_change_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<LocomotionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HyperdogTeleopJoy>());
  rclcpp::shutdown();
  return 0;
}
