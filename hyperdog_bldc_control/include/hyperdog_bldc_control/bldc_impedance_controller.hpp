// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// ros2_control controller that drives every joint like a BLDC actuator in
// "MIT mode" (impedance control running at the controller_manager rate):
//
//   tau_req = kp (q_des - q) + kd (dq_des - dq) + tau_ff
//   tau_out = BldcMotorModel(tau_req, dq)        (current / voltage / thermal limits)
//
// tau_out is written to the joint `effort` command interface. The motor model
// of every joint is configured from the parameter file (see
// hyperdog_description/config/bldc_motors.yaml).
//
// Topics:
//   ~/commands     (hyperdog_msgs/MotorCommands)  sub
//   ~/motor_states (hyperdog_msgs/MotorStates)    pub
// Safety: when no command arrives for `command_timeout` seconds the actuators
// switch to damping mode (kp = 0, kd = timeout_kd).

#ifndef HYPERDOG_BLDC_CONTROL__BLDC_IMPEDANCE_CONTROLLER_HPP_
#define HYPERDOG_BLDC_CONTROL__BLDC_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hyperdog_bldc_control/bldc_motor_model.hpp"
#include "hyperdog_msgs/msg/motor_commands.hpp"
#include "hyperdog_msgs/msg/motor_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hyperdog_bldc_control
{

struct JointCommand
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct CommandFrame
{
  std::vector<JointCommand> joints;
  bool valid{false};
};

class BldcImpedanceController : public controller_interface::ControllerInterface
{
public:
  BldcImpedanceController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  BldcMotorParams load_motor(const std::string & motor_type);
  void command_callback(const hyperdog_msgs::msg::MotorCommands::SharedPtr msg);

  std::vector<std::string> joints_;
  std::vector<BldcMotorModel> motors_;
  std::vector<double> lower_limits_, upper_limits_;
  double limit_stiffness_{50.0};
  bool simulate_dynamics_{true};
  double command_timeout_{0.25};
  double timeout_kd_{1.0};
  double state_publish_period_{0.01};
  double state_pub_age_{0.0};

  std::vector<double> last_torque_cmd_;
  std::shared_ptr<CommandFrame> last_frame_;
  double command_age_{0.0};

  realtime_tools::RealtimeBuffer<std::shared_ptr<CommandFrame>> command_buffer_;
  rclcpp::Subscription<hyperdog_msgs::msg::MotorCommands>::SharedPtr command_sub_;
  std::shared_ptr<rclcpp::Publisher<hyperdog_msgs::msg::MotorStates>> state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<hyperdog_msgs::msg::MotorStates>> rt_state_pub_;
};

}  // namespace hyperdog_bldc_control

#endif  // HYPERDOG_BLDC_CONTROL__BLDC_IMPEDANCE_CONTROLLER_HPP_
