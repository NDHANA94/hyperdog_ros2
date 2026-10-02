// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Declares every controller parameter on a ROS node and builds the
// ControllerConfig from them (names follow config/locomotion.yaml).

#ifndef ROS__PARAMETER_LOADER_HPP_
#define ROS__PARAMETER_LOADER_HPP_

#include "hyperdog_locomotion/controller_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hyperdog_locomotion
{

ControllerConfig load_controller_config(rclcpp::Node & node);

}  // namespace hyperdog_locomotion

#endif  // ROS__PARAMETER_LOADER_HPP_
