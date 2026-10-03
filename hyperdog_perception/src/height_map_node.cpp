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
// height_map_node: point clouds (depth camera, lidar) -> robot-centric elevation map.
//
// Subscribes:  every topic in `cloud_topics` (sensor_msgs/PointCloud2), TF
// Publishes:   hyperdog/height_map        (hyperdog_msgs/HeightMap, in `map_frame`)
//              hyperdog/height_map/cloud  (sensor_msgs/PointCloud2 of the cells, visualisation)

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hyperdog_msgs/msg/height_map.hpp"
#include "hyperdog_perception/elevation_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace hp = hyperdog_perception;

namespace
{
Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  T.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
    .normalized().toRotationMatrix();
  return T;
}
}  // namespace

class HeightMapNode : public rclcpp::Node
{
public:
  HeightMapNode()
  : Node("height_map")
  {
    hp::ElevationMapParams p;
    p.resolution = declare_parameter("resolution", p.resolution);
    p.length_x = declare_parameter("length_x", p.length_x);
    p.length_y = declare_parameter("length_y", p.length_y);
    p.fusion_alpha = declare_parameter("fusion_alpha", p.fusion_alpha);
    p.replace_threshold = declare_parameter("replace_threshold", p.replace_threshold);
    p.inpaint_radius = declare_parameter("inpaint_radius", p.inpaint_radius);
    map_ = std::make_unique<hp::ElevationMap>(p);

    map_frame_ = declare_parameter("map_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    const auto topics = declare_parameter(
      "cloud_topics", std::vector<std::string>{"/hyperdog/camera/points"});
    // per topic: frame to use instead of the header's (Gazebo's depth camera publishes x-forward
    // points with the optical frame id); empty: use the header
    auto overrides = declare_parameter("cloud_frame_overrides", std::vector<std::string>{""});
    overrides.resize(topics.size());
    decimation_ = std::max<int64_t>(1, declare_parameter("decimation", 4));
    max_range_ = declare_parameter("max_range", 3.0);
    // points relative to the base, outside [min_z, max_z] are dropped
    min_z_ = declare_parameter("min_z", -1.0);
    max_z_ = declare_parameter("max_z", 0.3);
    // self filter: box around the body in the base frame [x_min, x_max, y_min, y_max, z_min, z_max]
    self_box_ = declare_parameter(
      "self_filter_box", std::vector<double>{-0.35, 0.35, -0.2, 0.2, -0.18, 0.2});
    if (self_box_.size() != 6) {
      throw std::invalid_argument("self_filter_box needs 6 values");
    }
    const double rate = declare_parameter("publish_rate", 10.0);
    publish_cloud_ = declare_parameter("publish_cloud", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_pub_ = create_publisher<hyperdog_msgs::msg::HeightMap>("hyperdog/height_map", 5);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("hyperdog/height_map/cloud", 2);
    for (size_t k = 0; k < topics.size(); ++k) {
      const std::string frame = overrides[k];
      subs_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          topics[k], rclcpp::SensorDataQoS(),
          [this, frame](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {on_cloud(*m, frame);}));
      RCLCPP_INFO(get_logger(), "height map input: %s", topics[k].c_str());
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {publish();});
  }

private:
  bool lookup(const std::string & frame, const rclcpp::Time & stamp, Eigen::Isometry3d & T)
  {
    try {
      T = to_eigen(
        tf_buffer_->lookupTransform(
          map_frame_, frame, stamp, rclcpp::Duration::from_seconds(0.05)).transform);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF: %s", e.what());
      return false;
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & m, const std::string & frame_override)
  {
    const std::string frame = frame_override.empty() ? m.header.frame_id : frame_override;
    Eigen::Isometry3d T_map_sensor, T_map_base;
    if (!lookup(frame, m.header.stamp, T_map_sensor) ||
      !lookup(base_frame_, m.header.stamp, T_map_base))
    {
      return;
    }
    const Eigen::Isometry3d T_base_map = T_map_base.inverse();
    map_->move_to(T_map_base.translation().x(), T_map_base.translation().y());
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(m.width * m.height / static_cast<size_t>(decimation_) + 1);
    sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
    const size_t n = static_cast<size_t>(m.width) * m.height;
    for (size_t k = 0; k < n; ++k, ++x, ++y, ++z) {
      if (k % static_cast<size_t>(decimation_) != 0) {continue;}
      const Eigen::Vector3d ps(*x, *y, *z);
      if (!ps.allFinite() || ps.norm() > max_range_) {continue;}
      const Eigen::Vector3d pw = T_map_sensor * ps;
      const Eigen::Vector3d pb = T_base_map * pw;
      if (pb.z() < min_z_ || pb.z() > max_z_) {continue;}
      if (pb.x() > self_box_[0] && pb.x() < self_box_[1] && pb.y() > self_box_[2] &&
        pb.y() < self_box_[3] && pb.z() > self_box_[4] && pb.z() < self_box_[5])
      {
        continue;
      }
      pts.push_back(pw);
    }
    map_->integrate(pts);
    have_data_ = true;
  }

  void publish()
  {
    if (!have_data_) {return;}
    Eigen::Isometry3d T_map_base;
    if (lookup(base_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()), T_map_base)) {
      map_->move_to(T_map_base.translation().x(), T_map_base.translation().y());
    }
    hyperdog_msgs::msg::HeightMap hm;
    hm.header.stamp = now();
    hm.header.frame_id = map_frame_;
    hm.resolution = static_cast<float>(map_->resolution());
    hm.width = static_cast<uint32_t>(map_->width());
    hm.height = static_cast<uint32_t>(map_->height());
    hm.origin_x = map_->origin_x();
    hm.origin_y = map_->origin_y();
    hm.data = map_->inpainted();
    if (publish_cloud_ && cloud_pub_->get_subscription_count() > 0) {publish_cloud(hm);}
    map_pub_->publish(std::move(hm));
  }

  void publish_cloud(const hyperdog_msgs::msg::HeightMap & hm)
  {
    sensor_msgs::msg::PointCloud2 c;
    c.header = hm.header;
    sensor_msgs::PointCloud2Modifier mod(c);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t known = 0;
    for (float h : hm.data) {known += std::isnan(h) ? 0 : 1;}
    mod.resize(known);
    sensor_msgs::PointCloud2Iterator<float> x(c, "x"), y(c, "y"), z(c, "z");
    for (uint32_t iy = 0; iy < hm.height; ++iy) {
      for (uint32_t ix = 0; ix < hm.width; ++ix) {
        const float h = hm.data[iy * hm.width + ix];
        if (std::isnan(h)) {continue;}
        *x = static_cast<float>(hm.origin_x + (ix + 0.5) * hm.resolution);
        *y = static_cast<float>(hm.origin_y + (iy + 0.5) * hm.resolution);
        *z = h;
        ++x;
        ++y;
        ++z;
      }
    }
    cloud_pub_->publish(c);
  }

  std::unique_ptr<hp::ElevationMap> map_;
  std::string map_frame_, base_frame_;
  int64_t decimation_{4};
  double max_range_{3.0}, min_z_{-1.0}, max_z_{0.3};
  std::vector<double> self_box_;
  bool publish_cloud_{true}, have_data_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<hyperdog_msgs::msg::HeightMap>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeightMapNode>());
  rclcpp::shutdown();
  return 0;
}
