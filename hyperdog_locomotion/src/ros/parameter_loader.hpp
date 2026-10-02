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
