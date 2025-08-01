#ifndef SWERVE_VEL_CONVERTER_HPP
#define SWERVE_VEL_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <cmath>

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
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr wheel1_vel;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr wheel2_vel;
    
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr swerve0_pos;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr swerve1_pos;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr swerve2_pos;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    /*node value define end*/

    /*lifecycle callback begin*/
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    /*lifecycle callback end*/

    /*subscribe callback begin*/
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata);
    /*subscribe callback end*/

    /*parameter callback begin*/
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
    /*parameter callback end*/

    /*cal timer callback begin*/
    void cal_callback();
    /*cal timer callback end*/

    /*swerve drive cal begin*/
    std::vector<std::vector<double>> swerve_cal(const double Theta, const std::vector<double> &V);
    /*swerve drive cal end*/

    /*value declare begin*/
    double R; // robot radius
    double r; // wheel radius
    std::vector<double> v;
    double theta;
    /*value declare end*/
};

#endif