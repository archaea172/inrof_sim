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
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // constructor
    PursuitControler();
    //desconstructor
    ~PursuitControler();

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

    // omni_simulate
    void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3){
        const float a0 = M_PI/180*45;
        const float a1 = M_PI/180*135;
        const float a2 = M_PI/180*225;
        const float a3 = M_PI/180*315;
        const float r = 0.03;//m
        const float R = 0.144;//m
        float v[3] = {vx, vy, omega};
        float sint = sin(theta);
        float cost = cos(theta);

        float arr[4][3] =
        {{-cos(a0)*sint-sin(a0)*cost, cos(a0)*cost-sin(a0)*sint, R},
        {-cos(a1)*sint-sin(a1)*cost, cos(a1)*cost-sin(a1)*sint, R},
        {-cos(a2)*sint-sin(a2)*cost, cos(a2)*cost-sin(a2)*sint, R},
        {-cos(a3)*sint-sin(a3)*cost, cos(a3)*cost-sin(a3)*sint, R}};

        *w0 = (arr[0][0] * v[0] + arr[0][1] * v[1] + arr[0][2] * v[2]) / r;
        *w1 = (arr[1][0] * v[0] + arr[1][1] * v[1] + arr[1][2] * v[2]) / r;
        *w2 = (arr[2][0] * v[0] + arr[2][1] * v[1] + arr[2][2] * v[2]) / r;
        *w3 = (arr[3][0] * v[0] + arr[3][1] * v[1] + arr[3][2] * v[2]) / r;
    }

    // predict horizon
    int T;
    // sample
    int K;
    // control cycle
    double dt;
    // max value
    std::vector<double> max_value;
    // goal pose
    std::vector<double> goal_p;
    // current value
    std::vector<double> p;
    // vel ref
    std::vector<double> v_ref;
    
    float k_goal_angle;
    float k_goal_linear;
    float k_smooth_angle;
    float k_smooth_wheel;
    float k_smooth_linear;
    float k_vel_angle;
    float k_vel_linear;
    
    float iota;

    std::unique_ptr<MppiControl> mppi_controler;
    
    Eigen::Vector3d input_mu;
    Eigen::Matrix3d input_sigma;
};

#endif