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
    
    /*publisher create begin*/
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
    /*publisher create end*/

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
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_activate(const rclcpp_lifecycle::State &state)
{
    /*publisher on activate beign*/
    wheel0_vel->on_activate();
    wheel1_vel->on_activate();
    wheel2_vel->on_activate();
    swerve0_pos->on_activate();
    swerve1_pos->on_activate();
    swerve2_pos->on_activate();
    /*publisher on activate end*/

    /*subscriber and timer begin*/
    vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&SwerveVelConverter::vel_callback, this, _1)
    );
    cal_timer = this->create_wall_timer(0.001s, std::bind(&SwerveVelConverter::cal_callback, this));
    /*subscriber and timer end*/
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_deactivate(const rclcpp_lifecycle::State &state)
{
    /*node func reset begin*/
    wheel0_vel->on_deactivate();
    wheel1_vel->on_deactivate();
    wheel2_vel->on_deactivate();
    swerve0_pos->on_deactivate();
    swerve1_pos->on_deactivate();
    swerve2_pos->on_deactivate();

    vel_subscriber.reset();
    cal_timer.reset();
    /*node func reset begin*/
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "on error!");
    return CallbackReturn::SUCCESS;
}

SwerveVelConverter::CallbackReturn SwerveVelConverter::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
}
/*lifecycle callback end*/

/*parameter callback begin*/
rcl_interfaces::msg::SetParametersResult SwerveVelConverter::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}
/*parameter callback end*/