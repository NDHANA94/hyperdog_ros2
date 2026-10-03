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

#include "markers.hpp"

#include <string>
#include <vector>

namespace hyperdog_locomotion
{

namespace
{
using visualization_msgs::msg::Marker;

geometry_msgs::msg::Point point(const Vec3 & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker base(
  const std::string & frame, const rclcpp::Time & stamp, const std::string & ns, int id,
  int type)
{
  Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = rclcpp::Duration::from_seconds(0.5);
  return m;
}
}  // namespace

visualization_msgs::msg::MarkerArray make_markers(
  const Diagnostics & d, const std::string & frame, const rclcpp::Time & stamp,
  double force_scale)
{
  visualization_msgs::msg::MarkerArray arr;
  // estimated feet: red in contact, grey in the air
  Marker feet = base(frame, stamp, "feet", 0, Marker::SPHERE_LIST);
  feet.scale.x = feet.scale.y = feet.scale.z = 0.04;
  // foot targets: green (stance anchor / swing trajectory)
  Marker targets = base(frame, stamp, "foot_targets", 0, Marker::SPHERE_LIST);
  targets.scale.x = targets.scale.y = targets.scale.z = 0.025;
  targets.color = color(0.1f, 0.8f, 0.2f, 0.8f);
  // support polygon of the stance feet
  Marker polygon = base(frame, stamp, "support_polygon", 0, Marker::LINE_STRIP);
  polygon.scale.x = 0.006;
  polygon.color = color(1.0f, 0.6f, 0.0f);
  // stance order around the body for a closed polygon: FR, BR, BL, FL
  const int ring[4] = {0, 2, 3, 1};
  for (int k = 0; k < 4; ++k) {
    const int i = ring[k];
    const Vec3 f = d.feet_world.row(i).transpose();
    feet.points.push_back(point(f));
    feet.colors.push_back(d.contact[i] ? color(0.9f, 0.1f, 0.1f) : color(0.6f, 0.6f, 0.6f));
    targets.points.push_back(point(d.foot_target.row(i).transpose()));
    if (d.contact[i]) {polygon.points.push_back(point(f));}
  }
  if (polygon.points.size() > 2) {polygon.points.push_back(polygon.points.front());}
  arr.markers.push_back(feet);
  arr.markers.push_back(targets);
  arr.markers.push_back(polygon);
  // ground reaction forces (acting on the robot)
  for (int i = 0; i < 4; ++i) {
    Marker arrow = base(frame, stamp, "ground_reaction_forces", i, Marker::ARROW);
    arrow.scale.x = 0.01;
    arrow.scale.y = 0.02;
    arrow.scale.z = 0.02;
    arrow.color = color(0.1f, 0.4f, 1.0f);
    const Vec3 start = d.feet_world.row(i).transpose();
    const Vec3 f = d.foot_force.segment<3>(3 * i);
    arrow.points.push_back(point(start));
    arrow.points.push_back(point(start + force_scale * f));
    if (!d.contact[i] || f.norm() < 1e-3) {arrow.action = Marker::DELETE;}
    arr.markers.push_back(arrow);
  }
  return arr;
}

}  // namespace hyperdog_locomotion
