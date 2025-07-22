#include "pursuit.hpp"
#include "rclcpp/rclcpp.hpp"

/*class define begin*/
PursuitControler::PursuitControler()
: rclcpp_lifecycle::LifecycleNode(std::string("pursuit_controler"))
{
    /*parameter declare begin*/
    this->declare_parameter<double>("weight_goal_angle", 0);
    this->declare_parameter<double>("weight_goal_linear", 0);
    this->declare_parameter<double>("weight_smooth_angle", 0);
    this->declare_parameter<double>("weight_smooth_wheel", 0);
    this->declare_parameter<double>("weight_smooth_linear", 0);
    this->declare_parameter<double>("weight_vel_angle", 0);
    this->declare_parameter<double>("weight_vel_linear", 0);
    this->declare_parameter<double>("iota", 0);
    this->declare_parameter<int>("predict_horizon", 300);
    this->declare_parameter<int>("sampling_number", 100);
    this->declare_parameter<double>("control_cycle", 0.01);
    std::vector<double> max_vector(3, 1.0);
    this->declare_parameter("max_input_value", max_vector);
    /*parameter declare end*/

    /*parameter set begin*/
    k_goal_angle = this->get_parameter("weight_goal_angle").as_double();
    k_goal_linear = this->get_parameter("weight_goal_linear").as_double();
    k_smooth_angle = this->get_parameter("weight_smooth_angle").as_double();
    k_smooth_wheel = this->get_parameter("weight_smooth_wheel").as_double();
    k_smooth_linear = this->get_parameter("weight_smooth_linear").as_double();
    k_vel_angle = this->get_parameter("weight_vel_angle").as_double();
    k_vel_linear = this->get_parameter("weight_vel_linear").as_double();
    iota = this->get_parameter("iota").as_double();
    T = this->get_parameter("predict_horizon").as_int();
    K = this->get_parameter("sampling_number").as_int();
    dt = this->get_parameter("control_cycle").as_double();
    max_value = this->get_parameter("max_input_value").as_double_array();
    /*parameter set end*/

    /*sizing begin*/
    float input_dim = 3;
    p.resize(input_dim);
    v_ref.resize(2);
    /*sizing end*/
    
    mppi_controler = std::make_unique<MppiControl>(input_dim, this->K, this->T, this->max_value);

    parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&PursuitControler::parameters_callback, this, _1)
    );
}

PursuitControler::~PursuitControler()
{

}
/*class define end*/

/*lifecycle callback begin*/
PursuitControler::CallbackReturn PursuitControler::on_configure(const rclcpp_lifecycle::State &state)
{
    for (int i = 0; i < 3; i++) p[i] = 0;
    
    v_ref[0] = 1;
    v_ref[1] = 0.4;
    
    // m/s
    goal_p.resize(3);
    goal_p[0] = -1.2;
    goal_p[1] = 0;
    goal_p[2] = 0;

    /*create publisher*/
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"), rclcpp::SystemDefaultsQoS()
    );

    return CallbackReturn::SUCCESS;
}

PursuitControler::CallbackReturn PursuitControler::on_activate(const rclcpp_lifecycle::State &state)
{
    vel_publisher->on_activate();
    control_timer = this->create_wall_timer(0.01s, std::bind(&PursuitControler::control_callback, this));
    goal_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
        std::string("goal_pose"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&PursuitControler::goal_callback, this, _1)
    );
    pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
        std::string("pose"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&PursuitControler::pose_callback, this, _1)
    );
    return CallbackReturn::SUCCESS;
}

PursuitControler::CallbackReturn PursuitControler::on_deactivate(const rclcpp_lifecycle::State &state)
{
    vel_publisher->on_deactivate();
    control_timer.reset();
    goal_subscriber.reset();
    pose_subscriber.reset();

    return CallbackReturn::SUCCESS;
}

PursuitControler::CallbackReturn PursuitControler::on_cleanup(const rclcpp_lifecycle::State &state)
{
    vel_publisher.reset();
    return CallbackReturn::SUCCESS;
}

PursuitControler::CallbackReturn PursuitControler::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "on error!");
    return CallbackReturn::SUCCESS;
}

PursuitControler::CallbackReturn PursuitControler::on_shutdown(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}
/*lifecycle callback end*/

/*subscribe callback begin*/
void PursuitControler::goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
{
    this->goal_p[0] = rxdata->x;
    this->goal_p[1] = rxdata->y;
    this->goal_p[2] = rxdata->theta;
}

void PursuitControler::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
{
    this->p[0] = rxdata->x;
    this->p[1] = rxdata->y;
    this->p[2] = rxdata->theta;
}
/*subscribe callback end*/

/*parameter callback begin*/
rcl_interfaces::msg::SetParametersResult PursuitControler::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}
/*parameter callback end*/

/*control timer callback begin*/
void PursuitControler::control_callback()
{
    std::vector<double> input_array(3);
    this->mppi_controler->run(this->p);

    if (vel_publisher->is_activated())
    {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = input_array[0];
        txdata.linear.y = input_array[1];
        txdata.angular.z = input_array[2];
        vel_publisher->publish(txdata);
    }
}
/*control timer callback end*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PursuitControler> node = std::make_shared<PursuitControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}