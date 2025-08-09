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


#endif