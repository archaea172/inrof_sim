#include "swerve_vel_converter.hpp"
#include "rclcpp/rclcpp.hpp"

SwerveVelConverter::SwerveVelConverter()
: rclcpp_lifecycle::LifecycleNode(std::string("swerve_vel_converter"))
{
    /*parameter declare begin*/
    this->declare_parameter<double>("robot_radius", 1.0);
    this->declare_parameter<double>("wheel_radius", 0.1);
    /*parameter declare end*/
}