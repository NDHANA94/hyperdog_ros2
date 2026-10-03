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
// RViz visualization of the controller state: estimated feet (coloured by
// contact), foot targets, ground reaction forces and the support polygon.

#ifndef ROS__MARKERS_HPP_
#define ROS__MARKERS_HPP_

#include <string>

#include "hyperdog_locomotion/controller_types.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hyperdog_locomotion
{

/// force_scale: arrow length per Newton [m/N].
visualization_msgs::msg::MarkerArray make_markers(
  const Diagnostics & d, const std::string & frame, const rclcpp::Time & stamp,
  double force_scale = 0.004);

}  // namespace hyperdog_locomotion

#endif  // ROS__MARKERS_HPP_
