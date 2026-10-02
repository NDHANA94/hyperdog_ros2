// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// ROS 2 wrapper of the HyperDog locomotion controller.
//
// Subscriptions
//   joint_states            sensor_msgs/JointState     (joint_state_broadcaster)
//   imu                     sensor_msgs/Imu
//   foot contact topics     ros_gz_interfaces/Contacts (one per foot, optional)
//   cmd_vel                 geometry_msgs/Twist        (body frame vx, vy, wz)
//   hyperdog/command        hyperdog_msgs/LocomotionCommand
// Publications
//   bldc_controller/commands  hyperdog_msgs/MotorCommands (MIT mode)
//   hyperdog/state            hyperdog_msgs/LocomotionState
//   odom                      nav_msgs/Odometry (+ TF odom -> base_link)

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hyperdog_locomotion/locomotion_controller.hpp"
#include "parameter_loader.hpp"
#include "hyperdog_msgs/msg/locomotion_command.hpp"
#include "hyperdog_msgs/msg/locomotion_state.hpp"
#include "hyperdog_msgs/msg/motor_commands.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace hl = hyperdog_locomotion;
using namespace std::chrono_literals;

class LocomotionNode : public rclcpp::Node
{
public:
  LocomotionNode()
  : Node("locomotion_controller")
  {
    cfg_ = hl::load_controller_config(*this);
    controller_ = std::make_unique<hl::LocomotionController>(cfg_);
    names_ = hl::joint_names();

    auto_start_ = declare_parameter("locomotion.auto_start", true);
    start_mode_ = declare_parameter("locomotion.start_mode", std::string("locomotion"));
    auto_start_delay_ = declare_parameter("locomotion.auto_start_delay", 1.0);
    cmd_timeout_ = declare_parameter("locomotion.command_timeout", 0.5);
    contact_timeout_ = declare_parameter("estimation.contact_timeout", 0.012);
    publish_tf_ = declare_parameter("publish_tf", true);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    const double state_rate = declare_parameter("state_publish_rate", 50.0);
    const auto debug_file = declare_parameter("debug_log_file", std::string(""));
    if (!debug_file.empty()) {
      debug_.open(debug_file);
      debug_ << "t,mode,gait";
      for (const char * l : hl::kLegNames) {
        for (const char * k : {"c", "fx", "fy", "fz", "tx", "ty", "tz", "Fx", "Fy", "Fz"}) {
          debug_ << "," << l << "_" << k;
        }
      }
      debug_ << ",px,py,pz,vx,vy,vz,roll,pitch,yaw\n";
    }
    const auto contact_topics = declare_parameter(
      "foot_contact_topics", std::vector<std::string>{
      "hyperdog/foot_contact/FR", "hyperdog/foot_contact/FL",
      "hyperdog/foot_contact/BR", "hyperdog/foot_contact/BL"});
    command_.gait = declare_parameter("locomotion.default_gait", std::string("trot"));

    for (size_t i = 0; i < names_.size(); ++i) {joint_index_[names_[i]] = static_cast<int>(i);}

    auto sensor_qos = rclcpp::SensorDataQoS();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", sensor_qos, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (size_t k = 0; k < m->name.size(); ++k) {
          auto it = joint_index_.find(m->name[k]);
          if (it == joint_index_.end()) {continue;}
          const int i = it->second;
          if (k < m->position.size()) {sensors_.q[i] = m->position[k];}
          if (k < m->velocity.size()) {sensors_.dq[i] = m->velocity[k];}
          if (k < m->effort.size()) {sensors_.tau[i] = m->effort[k];}
        }
        have_joints_ = true;
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        const auto & o = m->orientation;
        if (m->orientation_covariance[0] >= 0.0 &&
        (o.x != 0.0 || o.y != 0.0 || o.z != 0.0 || o.w != 0.0))
        {
          sensors_.imu_orientation = Eigen::Quaterniond(o.w, o.x, o.y, o.z);
        } else {
          sensors_.imu_orientation.reset();
        }
        sensors_.gyro = hl::Vec3(
          m->angular_velocity.x, m->angular_velocity.y,
          m->angular_velocity.z);
        sensors_.accel = hl::Vec3(
          m->linear_acceleration.x, m->linear_acceleration.y,
          m->linear_acceleration.z);
        have_imu_ = true;
      });
    for (size_t i = 0; i < contact_topics.size() && i < 4; ++i) {
      contact_subs_.push_back(
        create_subscription<ros_gz_interfaces::msg::Contacts>(
          contact_topics[i], sensor_qos,
          [this, i](ros_gz_interfaces::msg::Contacts::ConstSharedPtr m) {
            if (m->contacts.empty()) {return;}
            std::lock_guard<std::mutex> lk(mtx_);
            last_contact_[i] = now();
            have_contact_sensor_ = true;
          }));
    }
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        command_.vx = m->linear.x;
        command_.vy = m->linear.y;
        command_.wz = m->angular.z;
        last_cmd_vel_ = now();
        have_cmd_vel_ = true;
      });
    hl_cmd_sub_ = create_subscription<hyperdog_msgs::msg::LocomotionCommand>(
      "hyperdog/command", 10, [this](hyperdog_msgs::msg::LocomotionCommand::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        switch (m->mode) {
          case hyperdog_msgs::msg::LocomotionCommand::MODE_PASSIVE: command_.mode =
            hl::Mode::PASSIVE; break;
          case hyperdog_msgs::msg::LocomotionCommand::MODE_STAND: command_.mode = hl::Mode::BALANCE;
            break;
          case hyperdog_msgs::msg::LocomotionCommand::MODE_LOCOMOTION: command_.mode =
            hl::Mode::LOCOMOTION; break;
          case hyperdog_msgs::msg::LocomotionCommand::MODE_SIT: command_.mode = hl::Mode::SIT;
            break;
          default: break;
        }
        if (!m->gait.empty()) {command_.gait = m->gait;}
        if (m->body_height > 0.0) {command_.body_height = m->body_height;}
        if (m->step_height > 0.0) {command_.step_height = m->step_height;}
        command_.body_rpy = hl::Vec3(m->body_rpy.x, m->body_rpy.y, m->body_rpy.z);
        auto_start_ = false;   // an explicit command overrides the auto start
      });

    motor_pub_ = create_publisher<hyperdog_msgs::msg::MotorCommands>(
      "bldc_controller/commands",
      10);
    state_pub_ = create_publisher<hyperdog_msgs::msg::LocomotionState>("hyperdog/state", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    state_decimation_ = std::max(
      1,
      static_cast<int>(cfg_.control_rate / std::max(state_rate, 1.0)));

    // the control loop follows the ROS clock (sim time in Gazebo)
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / cfg_.control_rate), [this]() {control_step();});
    RCLCPP_INFO(
      get_logger(), "HyperDog locomotion controller ready: %.0f Hz, balance=%s, contact=%s",
      cfg_.control_rate, cfg_.balance.controller.c_str(), cfg_.estimation.contact.source.c_str());
  }

private:
  void control_step()
  {
    hl::SensorData s;
    hl::Command cmd;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_joints_ || !have_imu_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "waiting for joint_states and imu ...");
        return;
      }
      s = sensors_;
      const auto t = now();
      if (have_contact_sensor_) {
        hl::Bool4 c;
        for (int i = 0; i < 4; ++i) {
          c[i] = (t - last_contact_[i]).seconds() < contact_timeout_;
        }
        s.foot_contact = c;
      }
      if (have_cmd_vel_ && (t - last_cmd_vel_).seconds() > cmd_timeout_) {
        command_.vx = command_.vy = command_.wz = 0.0;
      }
      if (auto_start_) {
        if (!ready_since_) {ready_since_ = t;}
        if ((t - *ready_since_).seconds() > auto_start_delay_) {
          command_.mode = start_mode_ == "stand" ? hl::Mode::BALANCE : hl::Mode::LOCOMOTION;
          auto_start_ = false;
        }
      }
      cmd = command_;
    }
    controller_->set_command(cmd);
    const hl::MotorCommand out = controller_->step(s);
    if (controller_->command().mode != cmd.mode) {
      // the controller changed the mode itself (fall protection, sit finished)
      std::lock_guard<std::mutex> lk(mtx_);
      command_.mode = controller_->command().mode;
    }
    for (const auto & [t, e] : controller_->take_events()) {
      RCLCPP_INFO(get_logger(), "[%.2fs] %s", t, e.c_str());
    }

    hyperdog_msgs::msg::MotorCommands m;
    m.header.stamp = now();
    m.name = names_;
    m.position.assign(out.q.data(), out.q.data() + 12);
    m.velocity.assign(out.dq.data(), out.dq.data() + 12);
    m.effort.assign(out.tau.data(), out.tau.data() + 12);
    m.kp.assign(out.kp.data(), out.kp.data() + 12);
    m.kd.assign(out.kd.data(), out.kd.data() + 12);
    motor_pub_->publish(m);

    if (++tick_ % state_decimation_ == 0) {publish_state(m.header.stamp);}
    if (debug_.is_open()) {
      const auto d = controller_->diagnostics();
      debug_ << now().seconds() << "," << hl::to_string(d.mode) << "," << d.gait;
      for (int i = 0; i < 4; ++i) {
        debug_ << "," << d.contact[i] << "," << d.feet_world(i, 0) << "," << d.feet_world(
          i,
          1) << "," <<
          d.feet_world(i, 2) << "," << d.foot_target(i, 0) << "," << d.foot_target(i, 1) << "," <<
          d.foot_target(
          i,
          2) << "," << d.foot_force[3 * i] << "," << d.foot_force[3 * i + 1] << "," <<
          d.foot_force[3 * i + 2];
      }
      debug_ << "," << d.position.x() << "," << d.position.y() << "," << d.position.z() << "," <<
        d.velocity.x() << "," << d.velocity.y() << "," << d.velocity.z() << "," << d.rpy.x() <<
        "," <<
        d.rpy.y() << "," << d.rpy.z() << "\n";
    }
  }

  void publish_state(const rclcpp::Time & stamp)
  {
    const auto d = controller_->diagnostics();
    hyperdog_msgs::msg::LocomotionState st;
    st.header.stamp = stamp;
    st.mode = hl::to_string(d.mode);
    st.gait = d.gait;
    st.balance_controller = d.controller;
    for (int i = 0; i < 4; ++i) {
      st.contact[i] = d.contact[i];
      st.scheduled_contact[i] = d.scheduled_contact[i];
      st.phase[i] = d.phase[i];
    }
    for (int i = 0; i < 12; ++i) {st.foot_force[i] = d.foot_force[i];}
    st.rpy.x = d.rpy.x(); st.rpy.y = d.rpy.y(); st.rpy.z = d.rpy.z();
    st.linear_velocity.x = d.velocity.x(); st.linear_velocity.y = d.velocity.y();
    st.linear_velocity.z = d.velocity.z();
    st.angular_velocity.x = d.omega.x(); st.angular_velocity.y = d.omega.y();
    st.angular_velocity.z = d.omega.z();
    st.body_height = d.height;
    st.disturbance_recovery = d.recovering;
    st.solve_time_ms = d.solve_time_ms;
    state_pub_->publish(st);

    if (d.mode == hl::Mode::BALANCE || d.mode == hl::Mode::LOCOMOTION) {
      const Eigen::Quaterniond q(hl::rpy_to_rot(d.rpy));
      nav_msgs::msg::Odometry od;
      od.header.stamp = stamp;
      od.header.frame_id = odom_frame_;
      od.child_frame_id = base_frame_;
      od.pose.pose.position.x = d.position.x();
      od.pose.pose.position.y = d.position.y();
      od.pose.pose.position.z = d.position.z();
      od.pose.pose.orientation.x = q.x();
      od.pose.pose.orientation.y = q.y();
      od.pose.pose.orientation.z = q.z();
      od.pose.pose.orientation.w = q.w();
      const hl::Vec3 v_body = hl::rpy_to_rot(d.rpy).transpose() * d.velocity;
      od.twist.twist.linear.x = v_body.x();
      od.twist.twist.linear.y = v_body.y();
      od.twist.twist.linear.z = v_body.z();
      od.twist.twist.angular.x = d.omega.x();
      od.twist.twist.angular.y = d.omega.y();
      od.twist.twist.angular.z = d.omega.z();
      odom_pub_->publish(od);
      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = od.header;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = d.position.x();
        tf.transform.translation.y = d.position.y();
        tf.transform.translation.z = d.position.z();
        tf.transform.rotation = od.pose.pose.orientation;
        tf_->sendTransform(tf);
      }
    }
  }

  hl::ControllerConfig cfg_;
  std::unique_ptr<hl::LocomotionController> controller_;
  std::vector<std::string> names_;
  std::map<std::string, int> joint_index_;

  std::mutex mtx_;
  hl::SensorData sensors_;
  hl::Command command_;
  bool have_joints_{false}, have_imu_{false}, have_contact_sensor_{false}, have_cmd_vel_{false};
  std::array<rclcpp::Time, 4> last_contact_{rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)};
  rclcpp::Time last_cmd_vel_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> ready_since_;

  bool auto_start_{true};
  std::string start_mode_;
  double auto_start_delay_{1.0};
  double cmd_timeout_{0.5};
  double contact_timeout_{0.012};
  bool publish_tf_{true};
  std::string odom_frame_, base_frame_;
  int state_decimation_{10};
  int64_t tick_{0};
  std::ofstream debug_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hyperdog_msgs::msg::LocomotionCommand>::SharedPtr hl_cmd_sub_;
  rclcpp::Publisher<hyperdog_msgs::msg::MotorCommands>::SharedPtr motor_pub_;
  rclcpp::Publisher<hyperdog_msgs::msg::LocomotionState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
