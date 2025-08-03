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

    /*value initialize begin*/
    this->theta = 0;
    this->v.resize(3);
    /*value initialize begin*/
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

/*subscribe callback begin*/
void SwerveVelConverter::vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    this->v[0] = rxdata->linear.x;
    this->v[1] = rxdata->linear.y;
    this->v[2] = rxdata->angular.z;
}
/*subscribe callback end*/

/*cal timer callback begin*/
void SwerveVelConverter::cal_callback()
{
    std::vector<std::vector<double>> swerve = this->swerve_control(this->swerve_cal(this->theta, this->v));
    std::vector<std::vector<std_msgs::msg::Float64>> txdata(2, std::vector<std_msgs::msg::Float64>(3));
    if(wheel0_vel->is_activated() && wheel1_vel->is_activated() && wheel2_vel->is_activated() && swerve0_pos->is_activated() && swerve1_pos->is_activated() && swerve2_pos->is_activated())
    {
        for (size_t i = 0; i < 2; i++) for (size_t j = 0; j < 3; j++) txdata[i][j].data = swerve[i][j];
        wheel0_vel->publish(txdata[0][0]);
        wheel1_vel->publish(txdata[0][1]);
        wheel2_vel->publish(txdata[0][2]);
        swerve0_pos->publish(txdata[1][0]);
        swerve1_pos->publish(txdata[1][1]);
        swerve2_pos->publish(txdata[1][2]);
    }
}
/*cal timer callback end*/

/*swerve drive cal begin*/
std::vector<std::vector<double>> SwerveVelConverter::swerve_cal(const double Theta, const std::vector<double> &V)
{
    std::vector<std::vector<double>> velocity(3, std::vector<double>(2, 0));
    std::vector<std::vector<double>> wheelandstare(2, std::vector<double>(3, 0));

    const float a[3] = {M_PI/6, 5*M_PI/6, 3*M_PI/2};

    for (int i = 0; i < 3; i++){
        velocity[i][0] = V[0] - V[2]*this->R*sin(Theta + a[i]);
        velocity[i][1] = V[1] + V[2]*this->R*cos(Theta + a[i]);
    }

    wheelandstare[0][0] = sqrt(pow(velocity[0][0], 2.0) + pow(velocity[0][1], 2.0))/r;
    wheelandstare[0][1] = sqrt(pow(velocity[1][0], 2.0) + pow(velocity[1][1], 2.0))/r;
    wheelandstare[0][2] = sqrt(pow(velocity[2][0], 2.0) + pow(velocity[2][1], 2.0))/r;


    wheelandstare[1][0] = atan2(velocity[0][1], velocity[0][0]);
    wheelandstare[1][1] = atan2(velocity[1][1], velocity[1][0]);
    wheelandstare[1][2] = atan2(velocity[2][1], velocity[2][0]);

    return wheelandstare;
}
std::vector<std::vector<double>> SwerveVelConverter::swerve_control(const std::vector<std::vector<double>> &swerve_num)
{
    std::vector<std::vector<double>> post_swerve(2, std::vector<double>(3, 0));
    return post_swerve;
}
/*swerve drive cal end*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SwerveVelConverter> node = std::make_shared<SwerveVelConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}