#ifndef PURSUIT_HPP
#define PURSUIT_HPP

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mppi_library.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PursuitControler : public rclcpp_lifecycle::LifecycleNode
{
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    PursuitControler();
    ~PursuitControler();
    /*class func end*/

private:
    /*node function begin*/
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;
    /*node function end*/

    /*lifecycle callback begin*/
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    /*lifecycle callback end*/

    /*subscribe callback begin*/
    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata);
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata);
    /*subscribe callback end*/

    /*parameter callback begin*/
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
    /*parameter callback end*/

    /*control timer callback begin*/
    void control_callback();
    /*control timer callback end*/

    /*mppi parameter begin*/
    std::vector<double> input_array;
    int T; // predict horizon
    int K; // sample number
    double dt; // control cycle
    std::vector<double> max_value; // max value
    std::vector<double> goal_p; // goal pose
    std::vector<double> p; // current value
    std::vector<double> v_ref; // vel ref
    double iota;
    std::unique_ptr<MppiControl> mppi_controler;
    Eigen::VectorXd input_mu;
    Eigen::MatrixXd input_sigma;
    
    /*weight begin*/
    float k_goal_angle;
    float k_goal_linear;
    float k_smooth_angle;
    float k_smooth_wheel;
    float k_smooth_linear;
    float k_vel_angle;
    float k_vel_linear;
    /*weight end*/
    
    /*mppi parameter end*/
};

#endif