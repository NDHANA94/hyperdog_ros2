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
// Closed-loop validation of HyperDog in Gazebo.
//
// Runs a scripted scenario against the full stack (Gazebo physics -> BLDC
// impedance controller -> locomotion controller) and grades it with the
// simulator ground truth:
//   * stand up and hold height
//   * external pushes while standing (lateral / frontal) -> must not fall,
//     must come back to rest (automatic stepping)
//   * velocity tracking: forward, lateral, turning, walk gait
//   * push while trotting
// A markdown report and a CSV time series are written; the process exits
// with 0 when every check passed.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hyperdog_msgs/msg/locomotion_command.hpp"
#include "hyperdog_msgs/msg/locomotion_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "scenarios.hpp"

namespace
{
struct Stats
{
  double max_tilt{0.0};
  double min_height{1e9};
  double max_height{-1e9};
  double sum_vx{0}, sum_vy{0};
  double yaw_prev{0}, yaw_accum{0}, t_track0{0}, t_track1{0};
  int n_lin{0};
  double sum_est_err2{0};
  int n_track{0};
  int n{0};
  double end_speed{0.0};
  bool fell{false};
  double end_tilt{0.0}, end_height{0.0};
  std::string end_mode;
  bool auto_step{false};
};

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
void rp_of(const geometry_msgs::msg::Quaternion & q, double & r, double & p)
{
  r = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  p = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}
}  // namespace

using hyperdog_gazebo::Step;

class ScenarioRunner : public rclcpp::Node
{
public:
  ScenarioRunner()
  : Node("scenario_runner")
  {
    report_file_ = declare_parameter("report_file", std::string("hyperdog_validation_report.md"));
    scenario_ = declare_parameter("scenario", std::string("full"));
    startup_timeout_ = declare_parameter("startup_timeout", 60.0);
    fall_tilt_ = declare_parameter("fall_tilt", 0.8);
    fall_height_ = declare_parameter("fall_height", 0.12);
    line_gain_y_ = declare_parameter("line_gain_y", 1.0);       // [1/s] hold_line steps
    line_gain_yaw_ = declare_parameter("line_gain_yaw", 1.5);   // [1/s]
    build_scenario();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    hl_pub_ = create_publisher<hyperdog_msgs::msg::LocomotionCommand>("hyperdog/command", 10);
    wrench_pub_ = create_publisher<ros_gz_interfaces::msg::EntityWrench>(
      "/world/hyperdog/wrench/persistent", 10);
    set_pose_ = create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/hyperdog/set_pose");
    clear_pub_ = create_publisher<ros_gz_interfaces::msg::Entity>(
      "/world/hyperdog/wrench/clear",
      10);
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/hyperdog/ground_truth", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {gt_ = *m; have_gt_ = true;});
    st_sub_ = create_subscription<hyperdog_msgs::msg::LocomotionState>(
      "hyperdog/state", 10,
      [this](hyperdog_msgs::msg::LocomotionState::ConstSharedPtr m) {
        state_ = *m; have_state_ = true;
      });
    csv_.open(report_file_ + ".csv");
    csv_ <<
      "t,step,x,y,z,roll,pitch,yaw,vx_body,vy_body,wz,est_vx,est_vy,"
      "mode,gait,recovering,solve_ms,est_height,est_roll,est_pitch\n";
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.01),
      [this]() {tick();});
  }

private:
  void build_scenario()
  {
    steps_ = hyperdog_gazebo::build_scenario(scenario_);
    if (steps_.empty()) {
      RCLCPP_ERROR(get_logger(), "unknown scenario '%s'", scenario_.c_str());
    }
    stats_.resize(steps_.size());
  }

  void send_push(double fx, double fy, double tx)
  {
    ros_gz_interfaces::msg::EntityWrench w;
    w.entity.name = "hyperdog::base_link";
    w.entity.type = ros_gz_interfaces::msg::Entity::LINK;
    w.wrench.force.x = fx;
    w.wrench.force.y = fy;
    w.wrench.torque.x = tx;
    wrench_pub_->publish(w);
  }
  void clear_push()
  {
    ros_gz_interfaces::msg::Entity e;
    e.name = "hyperdog::base_link";
    e.type = ros_gz_interfaces::msg::Entity::LINK;
    clear_pub_->publish(e);
  }

  void place(double roll, double height)
  {
    if (!set_pose_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "set_pose service not available: cannot place the robot");
      return;
    }
    auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    req->entity.name = "hyperdog";
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    req->pose.position.x = gt_.pose.pose.position.x;
    req->pose.position.y = gt_.pose.pose.position.y;
    req->pose.position.z = height;
    const double yaw = yaw_of(gt_.pose.pose.orientation);
    // q = Rz(yaw) * Rx(roll)
    req->pose.orientation.w = std::cos(yaw / 2) * std::cos(roll / 2);
    req->pose.orientation.x = std::cos(yaw / 2) * std::sin(roll / 2);
    req->pose.orientation.y = std::sin(yaw / 2) * std::sin(roll / 2);
    req->pose.orientation.z = std::sin(yaw / 2) * std::cos(roll / 2);
    set_pose_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "placing the robot: roll %.2f rad, height %.2f m", roll, height);
  }

  void tick()
  {
    const double t = now().seconds();
    if (finished_) {return;}
    if (!started_) {
      if (t0_ < 0.0) {t0_ = t;}
      if (have_gt_ && have_state_) {
        double r0, p0;
        rp_of(gt_.pose.pose.orientation, r0, p0);
        csv_ << t << ",-1," << gt_.pose.pose.position.x << "," << gt_.pose.pose.position.y << "," <<
          gt_.pose.pose.position.z << "," << r0 << "," << p0 << ",0," << gt_.twist.twist.linear.x <<
          "," <<
          gt_.twist.twist.linear.y << "," << gt_.twist.twist.angular.z << ",0,0," << state_.mode <<
          "," <<
          state_.gait << "," << state_.disturbance_recovery << "," << state_.solve_time_ms << "," <<
          state_.body_height << "," << state_.rpy.x << "," << state_.rpy.y << "\n";
      }
      // wait for the controller to stand up
      if (have_state_ && have_gt_ && (state_.mode == "LOCOMOTION" || state_.mode == "BALANCE")) {
        if (ready_t_ < 0.0) {ready_t_ = t;}
        if (t - ready_t_ > 2.0) {
          started_ = true;
          step_t0_ = t;
          stand_height_ = gt_.pose.pose.position.z;
          RCLCPP_INFO(
            get_logger(), "robot is up (height %.3f m) -> starting scenario '%s'",
            stand_height_, scenario_.c_str());
          hyperdog_msgs::msg::LocomotionCommand c;
          c.mode = hyperdog_msgs::msg::LocomotionCommand::MODE_LOCOMOTION;
          c.gait = "trot";
          hl_pub_->publish(c);
        }
      } else if (t - t0_ > startup_timeout_ && t0_ > 0.0) {
        startup_failed_ = true;
        finish();
      }
      return;
    }

    Step & s = steps_[step_];
    Stats & st = stats_[step_];
    const double ts = t - step_t0_;
    if (!std::isnan(s.place_roll) && placed_step_ != static_cast<int>(step_)) {
      place(s.place_roll, s.place_height);
      placed_step_ = static_cast<int>(step_);
    }
    // commands
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = s.vx;
    cmd.linear.y = s.vy;
    cmd.angular.z = s.wz;
    if (s.hold_line) {
      // operator: steer back to the line y = 0 with heading 0
      const double y_err = gt_.pose.pose.position.y;
      const double yaw_err = yaw_of(gt_.pose.pose.orientation);
      cmd.linear.y -= std::clamp(line_gain_y_ * y_err, -0.15, 0.15);
      cmd.angular.z -= std::clamp(line_gain_yaw_ * yaw_err, -0.4, 0.4);
    }
    cmd_pub_->publish(cmd);
    if (gait_sent_ != s.gait) {
      hyperdog_msgs::msg::LocomotionCommand c;
      c.mode = hyperdog_msgs::msg::LocomotionCommand::MODE_LOCOMOTION;
      c.gait = s.gait;
      hl_pub_->publish(c);
      gait_sent_ = s.gait;
    }
    for (size_t k = 0; k < s.pushes.size(); ++k) {
      const auto & p = s.pushes[k];
      const int key = static_cast<int>(step_ * 10 + k);
      if (ts >= p.start && push_active_ != key && push_done_.count(key) == 0) {
        send_push(p.fx, p.fy, p.tx);
        push_active_ = key;
        RCLCPP_INFO(
          get_logger(), "push: F = (%.0f, %.0f) N, Mx = %.0f Nm for %.2f s", p.fx, p.fy, p.tx,
          p.duration);
      }
      if (push_active_ == key && ts >= p.start + p.duration) {
        clear_push();
        push_active_ = -1;
        push_done_.insert(key);
      }
    }

    // metrics from ground truth
    double r, pch;
    rp_of(gt_.pose.pose.orientation, r, pch);
    const double yaw = yaw_of(gt_.pose.pose.orientation);
    const double c = std::cos(yaw), sn = std::sin(yaw);
    // gz odometry twist is expressed in the body frame (OdometryPublisher)
    const double vxb = gt_.twist.twist.linear.x;
    const double vyb = gt_.twist.twist.linear.y;
    const double wz = gt_.twist.twist.angular.z;   // logged only (can contain spikes)
    const double z = gt_.pose.pose.position.z;
    (void)c; (void)sn;
    st.max_tilt = std::max({st.max_tilt, std::abs(r), std::abs(pch)});
    st.min_height = std::min(st.min_height, z);
    st.max_height = std::max(st.max_height, z);
    st.auto_step = st.auto_step || state_.disturbance_recovery;
    st.n++;
    // estimator error (body frame velocity)
    const double ecy = std::cos(state_.rpy.z), esy = std::sin(state_.rpy.z);
    const double est_vxb = ecy * state_.linear_velocity.x + esy * state_.linear_velocity.y;
    const double est_vyb = -esy * state_.linear_velocity.x + ecy * state_.linear_velocity.y;
    st.sum_est_err2 += (est_vxb - vxb) * (est_vxb - vxb) + (est_vyb - vyb) * (est_vyb - vyb);
    // Gazebo's odometry publisher occasionally outputs bogus twist spikes: the yaw rate is
    // computed from the unwrapped heading change and implausible linear samples are skipped
    if (s.track_from >= 0.0 && ts >= s.track_from) {
      if (st.n_track == 0) {
        st.yaw_prev = yaw;
        st.t_track0 = t;
      }
      st.yaw_accum += std::remainder(yaw - st.yaw_prev, 2.0 * M_PI);
      st.yaw_prev = yaw;
      st.t_track1 = t;
      if (std::abs(vxb) < 3.0 && std::abs(vyb) < 3.0) {
        st.sum_vx += vxb;
        st.sum_vy += vyb;
        st.n_lin++;
      }
      st.n_track++;
    }
    st.end_speed = std::hypot(vxb, vyb);
    st.end_tilt = std::max(std::abs(r), std::abs(pch));
    st.end_height = z;
    st.end_mode = state_.mode;
    if (std::abs(r) > fall_tilt_ || std::abs(pch) > fall_tilt_ || z < fall_height_) {
      if (s.allow_fall) {
        if (!st.fell) {
          RCLCPP_INFO(get_logger(), "robot is down (expected in '%s')", s.name.c_str());
        }
        st.fell = true;
      } else {
        fell_ = true;
        fall_step_ = step_;
      }
    }
    csv_ << t << "," << step_ << "," << gt_.pose.pose.position.x << "," <<
      gt_.pose.pose.position.y << "," << z <<
      "," << r << "," << pch << "," << yaw << "," << vxb << "," << vyb << "," << wz << "," <<
      est_vxb << "," <<
      est_vyb << "," << state_.mode << "," << state_.gait << "," << state_.disturbance_recovery <<
      "," <<
      state_.solve_time_ms << "," << state_.body_height << "," << state_.rpy.x << "," <<
      state_.rpy.y << "\n";

    if (fell_) {
      RCLCPP_ERROR(get_logger(), "robot FELL during '%s'", s.name.c_str());
      finish();
      return;
    }
    if (ts >= s.duration) {
      RCLCPP_INFO(get_logger(), "step '%s' done", s.name.c_str());
      ++step_;
      step_t0_ = t;
      if (step_ >= steps_.size()) {finish();}
    }
  }

  void finish()
  {
    finished_ = true;
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
    clear_push();
    bool all_ok = !fell_ && !startup_failed_;
    std::ostringstream md;
    md << "# HyperDog closed-loop validation report\n\n";
    md << "Distance travelled (ground truth): x = " << gt_.pose.pose.position.x << " m, y = " <<
      gt_.pose.pose.position.y << " m\n\n";
    md << "Scenario: `" << scenario_ << "` - simulator: Gazebo Harmonic (DART, 1 kHz), "
       <<
      "actuators: simulated BLDC (MIT impedance mode, 1 kHz), "
      "controller: hyperdog_locomotion (500 Hz)\n\n";
    if (startup_failed_) {
      md << "**FAILED: the robot did not reach a standing mode within " << startup_timeout_ <<
        " s.**\n";
    } else {
      md << "Standing height after stand-up: " << stand_height_ << " m\n\n";
      md << "| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | "
        "auto-step | est. vel RMSE [m/s] |\n|---|---|---|---|---|---|---|\n";
      for (size_t i = 0; i < steps_.size() && i < stats_.size(); ++i) {
        const Step & s = steps_[i];
        const Stats & st = stats_[i];
        if (st.n == 0) {
          md << "| " << s.name << " | not run | | | | | |\n";
          continue;
        }
        bool ok = !(fell_ && fall_step_ == i);
        std::ostringstream trk;
        if (s.track_from >= 0.0 && st.n_track > 0) {
          const double vx = st.sum_vx / std::max(1, st.n_lin),
            vy = st.sum_vy / std::max(1, st.n_lin);
          const double wz = st.yaw_accum / std::max(1e-3, st.t_track1 - st.t_track0);
          trk.precision(2);
          trk << std::fixed << "vx " << s.vx << "->" << vx << ", vy " << s.vy << "->" << vy <<
            ", wz " << s.wz <<
            "->" << wz;
          ok = ok && std::abs(vx - s.vx) < s.vx_tol && std::abs(vy - s.vy) < s.vy_tol &&
            std::abs(wz - s.wz) < s.wz_tol;
        }
        if (s.expect_rest_at_end) {
          trk << "end speed " << std::fixed << st.end_speed << " m/s";
          ok = ok && st.end_speed < 0.1;
        }
        if (s.allow_fall) {
          // must be standing upright again: small tilt, near the standing height, active mode
          const bool up = st.end_tilt<0.2 && st.end_height>0.8 * stand_height_ &&
            (st.end_mode == "BALANCE" || st.end_mode == "LOCOMOTION");
          if (!trk.str().empty()) {trk << ", ";}
          trk << (st.fell ? "fell" : "did not fall") << ", end: " << st.end_mode << " tilt " <<
            std::fixed << std::setprecision(1) << st.end_tilt * 180.0 / M_PI << " deg";
          ok = ok && up && st.fell;   // the scenario is meaningless without the fall
        } else {
          ok = ok && st.max_tilt < 0.5;
        }
        all_ok = all_ok && ok;
        md.precision(3);
        md << "| " << s.name << " | " << (ok ? "PASS" : "**FAIL**") << " | " << std::fixed <<
          st.max_tilt * 180.0 / M_PI << " | " << st.min_height << " / " << st.max_height << " | " <<
          trk.str() << " | " << (st.auto_step ? "yes" : "no") << " | " <<
          std::sqrt(st.sum_est_err2 / st.n) << " |\n";
      }
      if (fell_) {md << "\n**The robot fell during step: " << steps_[fall_step_].name << "**\n";}
    }
    md << "\n**Overall: " << (all_ok ? "PASS" : "FAIL") << "**\n";
    std::ofstream f(report_file_);
    f << md.str();
    f.close();
    csv_.close();
    RCLCPP_INFO(get_logger(), "\n%s\nreport written to %s", md.str().c_str(), report_file_.c_str());
    exit_code_ = all_ok ? 0 : 1;
    rclcpp::shutdown();
  }

public:
  int exit_code_{1};

private:
  std::string report_file_, scenario_;
  double startup_timeout_, fall_tilt_, fall_height_;
  double line_gain_y_{1.0}, line_gain_yaw_{1.5};
  std::vector<Step> steps_;
  std::vector<Stats> stats_;
  size_t step_{0};
  double t0_{-1.0}, ready_t_{-1.0}, step_t0_{0.0}, stand_height_{0.0};
  bool started_{false}, finished_{false}, fell_{false}, startup_failed_{false};
  size_t fall_step_{0};
  int push_active_{-1};
  std::set<int> push_done_;
  std::string gait_sent_;
  nav_msgs::msg::Odometry gt_;
  hyperdog_msgs::msg::LocomotionState state_;
  bool have_gt_{false}, have_state_{false};
  std::ofstream csv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<hyperdog_msgs::msg::LocomotionCommand>::SharedPtr hl_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Entity>::SharedPtr clear_pub_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_;
  int placed_step_{-1};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<hyperdog_msgs::msg::LocomotionState>::SharedPtr st_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScenarioRunner>();
  rclcpp::spin(node);
  const int code = node->exit_code_;
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
