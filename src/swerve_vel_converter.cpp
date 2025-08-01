#include "swerve_vel_converter.hpp"
#include "rclcpp/rclcpp.hpp"

SwerveVelConverter::SwerveVelConverter()
: rclcpp_lifecycle::LifecycleNode(std::string("swerve_vel_converter"))
{
    /*parameter declare begin*/
    this->declare_parameter<double>("robot_radius", 1.0);
    this->declare_parameter<double>("wheel_radius", 0.1);
    /*parameter declare end*/

    /*parameter set begin*/
    this->R = this->get_parameter("robot_radius").as_double();
    this->r = this->get_parameter("wheel_radius").as_double();
    /*parameter set end*/
}