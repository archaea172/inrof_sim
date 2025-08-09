#ifndef MPPI_SWERVE_HPP
#define MPPI_SWERVE_HPP

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

/*mppi class begin*/
class MppiSwerve : public MppiControl
{
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    MppiSwerve();
    ~MppiSwerve();
    /*class func end*/

private:
    /*declare value begin*/
    double R; // robot radius
    double r; // wheel radius
    Eigen::VectorXd gain_vector;
    int num_evaluation;
    /*declare value end*/

    /*evaluation func begin*/
    double calc_evaluation(Eigen::MatrixXd InputList, Eigen::MatrixXd StateList, Eigen::MatrixXd GoalPose) override;
    /*evaluation func end*/

    /*model func begin*/
    Eigen::VectorXd model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state) override;
    Eigen::MatrixXd generate_model_state(const Eigen::MatrixXd &input_array, const Eigen::VectorXd &init_state) override;
    /*model func end*/

    /*swerve drive sim begin*/
    std::vector<std::vector<double>> swerve_cal(const double Theta, const std::vector<double> &V);
    std::vector<std::vector<double>> swerve_control(const std::vector<std::vector<double>> &swerve_num);
    /*swerve drive sim end*/
};
/*mppi class end*/

/*node begin*/
class MPPISwerveNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    MPPISwerveNode();
    ~MPPISwerveNode();
    /*class func end*/

private:
    /*value declare begin*/

    /*weight begin*/
    double k_goal_angle;
    double k_goal_linear;
    double k_smooth_angle_vel;
    double k_smooth_linear_vel;
    double k_smooth_wheel;
    double k_smooth_stare;
    /*weight end*/
    
    /*node function begin*/
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;
    /*node function end*/
    
    /*value declare end*/
};
/*node end*/

#endif