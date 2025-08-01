#include "swerve_vel_converter.hpp"
#include "rclcpp/rclcpp.hpp"

/*class func begin*/
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
    
    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SwerveVelConverter::parameters_callback, this, _1)
    );
}

SwerveVelConverter::~SwerveVelConverter()
{

}
/*class func end*/

/*lifecycle callback begin*/
SwerveVelConverter::CallbackReturn SwerveVelConverter::on_configure(const rclcpp_lifecycle::State &state)
{
    wheel0_vel = this->create_publisher<std_msgs::msg::Float64>(
        std::string("wheel_vel0"), rclcpp::SystemDefaultsQoS()
    );
    wheel1_vel = this->create_publisher<std_msgs::msg::Float64>(
        std::string("wheel_vel1"), rclcpp::SystemDefaultsQoS()
    );
    wheel2_vel = this->create_publisher<std_msgs::msg::Float64>(
        std::string("wheel_vel2"), rclcpp::SystemDefaultsQoS()
    );

    swerve0_pos = this->create_publisher<std_msgs::msg::Float64>(
        std::string("swerve_pos0"), rclcpp::SystemDefaultsQoS()
    );
    swerve1_pos = this->create_publisher<std_msgs::msg::Float64>(
        std::string("swerve_pos1"), rclcpp::SystemDefaultsQoS()
    );
    swerve2_pos = this->create_publisher<std_msgs::msg::Float64>(
        std::string("swerve_pos2"), rclcpp::SystemDefaultsQoS()
    );
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_activate(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}
/*lifecycle callback end*/