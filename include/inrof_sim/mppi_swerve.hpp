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

    /*main progress begin*/
    std::vector<double> run(
        std::vector<double> &init_state,
        std::vector<double> &goal_state,
        Eigen::VectorXd &mu,
        Eigen::MatrixXd &sigma,
        double iota
    );
    /*main progress end*/

private:

};

#endif