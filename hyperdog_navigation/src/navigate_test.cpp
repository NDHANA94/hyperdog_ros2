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
// navigate_test: sends one NavigateToPose goal (odom frame) and grades the result.
//
//   ros2 run hyperdog_navigation navigate_test --ros-args -p use_sim_time:=true
//     -p goal_x:=5.0 -p goal_y:=0.0 -p goal_yaw:=0.0 -p timeout:=120.0
//
// PASS when Nav2 reports success and, if a ground truth odometry topic is available
// (simulation), the true final position is within `tolerance` of the goal. A robot
// that falls (ground truth tilt above 45 deg) fails immediately. Exit code 0 on PASS.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateTest : public rclcpp::Node
{
public:
  NavigateTest()
  : Node("navigate_test")
  {
    goal_x_ = declare_parameter("goal_x", 5.0);
    goal_y_ = declare_parameter("goal_y", 0.0);
    goal_yaw_ = declare_parameter("goal_yaw", 0.0);
    timeout_ = declare_parameter("timeout", 120.0);
    tolerance_ = declare_parameter("tolerance", 0.4);
    frame_ = declare_parameter("frame", std::string("odom"));
    const auto gt_topic = declare_parameter(
      "ground_truth_topic",
      std::string("/hyperdog/ground_truth"));
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gt_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        gt_ = *m;
        have_gt_ = true;
        const auto & q = m->pose.pose.orientation;
        const double tilt = std::acos(
          std::clamp(1.0 - 2.0 * (q.x * q.x + q.y * q.y), -1.0, 1.0));
        max_tilt_ = std::max(max_tilt_, tilt);
      });
  }

  int run()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(60))) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      return 1;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = goal_x_;
    goal.pose.pose.position.y = goal_y_;
    goal.pose.pose.orientation.z = std::sin(goal_yaw_ / 2.0);
    goal.pose.pose.orientation.w = std::cos(goal_yaw_ / 2.0);
    RCLCPP_INFO(
      get_logger(), "goal: (%.2f, %.2f, %.2f rad) in %s", goal_x_, goal_y_, goal_yaw_,
      frame_.c_str());
    auto gh_future = client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(
        shared_from_this(), gh_future,
        std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS || !gh_future.get())
    {
      RCLCPP_ERROR(get_logger(), "goal rejected");
      return 1;
    }
    auto result_future = client_->async_get_result(gh_future.get());
    const auto t0 = now();
    while (rclcpp::ok()) {
      if (rclcpp::spin_until_future_complete(
          shared_from_this(), result_future,
          std::chrono::milliseconds(100)) == rclcpp::FutureReturnCode::SUCCESS)
      {
        break;
      }
      if (max_tilt_ > M_PI / 4.0) {
        RCLCPP_ERROR(get_logger(), "FAIL: the robot fell");
        client_->async_cancel_all_goals();
        return 1;
      }
      if ((now() - t0).seconds() > timeout_) {
        RCLCPP_ERROR(get_logger(), "FAIL: timeout after %.0f s", timeout_);
        client_->async_cancel_all_goals();
        return 1;
      }
    }
    const auto result = result_future.get();
    const double dt = (now() - t0).seconds();
    const bool nav_ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    bool ok = nav_ok;
    std::string gt_text = "no ground truth";
    if (have_gt_) {
      const double err = std::hypot(
        gt_.pose.pose.position.x - goal_x_, gt_.pose.pose.position.y - goal_y_);
      ok = ok && err < tolerance_;
      gt_text = "true position (" + std::to_string(gt_.pose.pose.position.x).substr(0, 5) + ", " +
        std::to_string(gt_.pose.pose.position.y).substr(0, 5) + "), error " +
        std::to_string(err).substr(0, 5) + " m";
    }
    RCLCPP_INFO(
      get_logger(), "%s: Nav2 result %s after %.1f s; %s; max tilt %.1f deg",
      ok ? "PASS" : "FAIL", nav_ok ? "SUCCEEDED" : "not succeeded", dt, gt_text.c_str(),
      max_tilt_ * 180.0 / M_PI);
    return ok ? 0 : 1;
  }

private:
  double goal_x_, goal_y_, goal_yaw_, timeout_, tolerance_;
  std::string frame_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  nav_msgs::msg::Odometry gt_;
  bool have_gt_{false};
  double max_tilt_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateTest>();
  const int code = node->run();
  rclcpp::shutdown();
  return code;
}
