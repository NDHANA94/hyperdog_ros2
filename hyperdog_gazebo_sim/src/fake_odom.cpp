#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>


#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>


#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hyperdog_msgs/msg/joy_ctrl_cmds.hpp"


using std::placeholders::_1;

class FakeOdom : public rclcpp::Node
{
    public: 
        FakeOdom()
        : Node("Fake_odom_node")
        {
            sub_ = this->create_subscription<hyperdog_msgs::msg::JoyCtrlCmds>(
                "hyperdog_joy_ctrl_cmd", 30, std::bind(&FakeOdom::topic_callback, this, _1));

            pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
        }
    
    private:
        
        void topic_callback(const hyperdog_msgs::msg::JoyCtrlCmds::SharedPtr msg) const
        {   auto odom = nav_msgs::msg::Odometry();
            float x;
            float y;
            bool side_walk;
            x = msg->gait_step.x;
            y = msg->gait_step.y;
            side_walk = msg->states[2];
            std::cout<< x << " " << y << " " << side_walk << "\n";
        }
        rclcpp::Subscription<hyperdog_msgs::msg::JoyCtrlCmds>::SharedPtr sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdom>());
    rclcpp::shutdown();
    return 0;
}