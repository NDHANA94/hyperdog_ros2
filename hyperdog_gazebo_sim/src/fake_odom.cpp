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

double odom_x = 0;
double odom_y = 0;
double odom_rot = 0;
double pi = 3.14159;

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
            double x;
            double y;
            // double rot;
            bool side_walk;
            x = msg->gait_step.x/1000;
            y = msg->gait_step.y/1000;
            side_walk = msg->states[2];
            if (!side_walk){
                odom_x += x;
                odom_y += y;
            }
            if (side_walk && y > 0){
                odom_x += x;
                odom_y += y/2;
                odom_rot += y;
                if (odom_rot >= 2*pi){
                    odom_rot = odom_rot - 2*pi;
                }
                else if (odom_rot <= -2*pi){
                    odom_rot = odom_rot + 2*pi;
                }
            }
            odom.pose.pose.position.x = odom_x;
            odom.pose.pose.position.y = odom_y;
            odom.pose.pose.orientation.z = odom_rot;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            // std::cout<< x << " " << y << " " << side_walk << "\n";
            pub_ -> publish(odom);
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