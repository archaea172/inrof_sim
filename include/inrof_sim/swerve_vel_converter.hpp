#ifndef SWERVE_VEL_CONVERTER_HPP
#define SWERVE_VEL_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SwerveVelConverter : public rclcpp_lifecycle::LifecycleNode
{
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    SwerveVelConverter();
    ~SwerveVelConverter();
    /*class func end*/

private:
    /*node value define begin*/
    rclcpp::TimerBase::SharedPtr cal_timer;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr wheel0_vel;
    /*node value define end*/
};

#endif