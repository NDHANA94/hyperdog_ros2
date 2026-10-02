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
// ros2_control SystemInterface for real BLDC actuators speaking the MIT CAN
// protocol over Linux SocketCAN. Exposes position/velocity/effort state and an
// effort command per joint; the BldcImpedanceController (with
// simulate_motor_dynamics: false) closes the impedance loop on top of it, so
// the same controller stack runs in simulation and on the robot.
//
// URDF <ros2_control> parameters:
//   hardware:  can_interface (default can0)
//   joint:     can_id, direction (+1/-1), offset [rad],
//              p_max, v_max, kp_max, kd_max, t_max (MIT packing ranges)

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hyperdog_bldc_control/mit_can_protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace hyperdog_bldc_control
{

class MitCanSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto get = &MitCanSystem::param_or;
    auto it = info_.hardware_parameters.find("can_interface");
    can_if_ = it == info_.hardware_parameters.end() ? "can0" : it->second;
    const size_t n = info_.joints.size();
    ids_.resize(n);
    dir_.resize(n);
    offset_.resize(n);
    ranges_.resize(n);
    pos_.assign(n, 0.0);
    vel_.assign(n, 0.0);
    eff_.assign(n, 0.0);
    cmd_.assign(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
      const auto & p = info_.joints[i].parameters;
      ids_[i] = static_cast<int>(get(p, "can_id", static_cast<double>(i + 1)));
      dir_[i] = get(p, "direction", 1.0) >= 0.0 ? 1.0 : -1.0;
      offset_[i] = get(p, "offset", 0.0);
      ranges_[i].p_max = get(p, "p_max", 12.5);
      ranges_[i].v_max = get(p, "v_max", 50.0);
      ranges_[i].kp_max = get(p, "kp_max", 500.0);
      ranges_[i].kd_max = get(p, "kd_max", 5.0);
      ranges_[i].t_max = get(p, "t_max", 25.0);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    sock_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(logger(), "socket(): %s", std::strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, can_if_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(logger(), "CAN interface '%s' not found", can_if_.c_str());
      close(sock_);
      sock_ = -1;
      return hardware_interface::CallbackReturn::ERROR;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(logger(), "bind(): %s", std::strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> s;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      s.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
      s.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
      s.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &eff_[i]);
    }
    return s;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> c;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      c.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &cmd_[i]);
    }
    return c;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < ids_.size(); ++i) {
      send(ids_[i], mit_enter_motor_mode());
      cmd_[i] = 0.0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < ids_.size(); ++i) {
      send(ids_[i], mit_pack_command(ranges_[i], 0, 0, 0, 0, 0));
      send(ids_[i], mit_exit_motor_mode());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    struct can_frame f {};
    while (sock_ >= 0 && ::read(sock_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f))) {
      if (f.can_dlc < 6) {continue;}
      const uint8_t id = f.data[0];
      for (size_t i = 0; i < ids_.size(); ++i) {
        if (ids_[i] != id) {continue;}
        const auto fb = mit_unpack_reply(ranges_[i], f.data, f.can_dlc);
        pos_[i] = dir_[i] * fb.position - offset_[i];
        vel_[i] = dir_[i] * fb.velocity;
        eff_[i] = dir_[i] * fb.torque;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (size_t i = 0; i < ids_.size(); ++i) {
      // pure torque mode: the impedance law runs in BldcImpedanceController
      send(ids_[i], mit_pack_command(ranges_[i], 0.0, 0.0, 0.0, 0.0, dir_[i] * cmd_[i]));
    }
    return hardware_interface::return_type::OK;
  }

private:
  static double param_or(
    const std::unordered_map<std::string, std::string> & m, const std::string & key, double def)
  {
    const auto it = m.find(key);
    return it == m.end() ? def : std::stod(it->second);
  }

  rclcpp::Logger logger() const {return rclcpp::get_logger("MitCanSystem");}

  void send(int id, const std::array<uint8_t, 8> & data)
  {
    if (sock_ < 0) {return;}
    struct can_frame f {};
    f.can_id = static_cast<canid_t>(id);
    f.can_dlc = 8;
    std::memcpy(f.data, data.data(), 8);
    if (::write(sock_, &f, sizeof(f)) != static_cast<ssize_t>(sizeof(f))) {
      RCLCPP_WARN_THROTTLE(logger(), clock_, 2000, "CAN write failed for id %d", id);
    }
  }

  std::string can_if_;
  int sock_{-1};
  std::vector<int> ids_;
  std::vector<double> dir_, offset_;
  std::vector<MitRanges> ranges_;
  std::vector<double> pos_, vel_, eff_, cmd_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace hyperdog_bldc_control

#pragma GCC diagnostic pop

PLUGINLIB_EXPORT_CLASS(hyperdog_bldc_control::MitCanSystem, hardware_interface::SystemInterface)
