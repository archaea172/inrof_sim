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
    std::vector<double> max_value_d = this->get_parameter("max_input_value").as_double_array();
    std::transform(max_value_d.begin(), max_value_d.end(), std::back_inserter(max_value),
    [](double d){ return static_cast<float>(d);});
    /*parameter set end*/

    /*sizing begin*/
    p.resize(3);
    v_ref.resize(2);
    max_value.resize(3);
    /*sizing end*/

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
    std::vector<std::vector<std::vector<float>>> all_v_array(K, std::vector<std::vector<float>>(T, std::vector<float>(3, 0)));
    std::vector<float> S_array(K, 0);
    // sampling
    for (size_t i = 0; i < K; i++)
    {
        std::vector<std::vector<float>> v_array = generate_v_array(3, max_value);
        std::vector<std::vector<float>> p_array = predict_position_array(this->p, v_array);
        float S = k_goal_angle*estimate_goal_angle(p_array)
        + k_goal_linear*estimate_goal_linear(p_array) + k_smooth_angle*estimate_smooth_rotate(v_array) 
        + k_smooth_wheel*estimate_smooth_wheel(p_array, v_array) + k_smooth_linear*estimate_smooth_vel(v_array) 
        + k_vel_linear*estimate_vel(v_array) + k_vel_angle*estimate_vel_rotate(v_array);
        all_v_array[i] = v_array;
        S_array[i] = S;
    }
    // calc weight
    std::vector<float> weight(K);
    float S_ref = *std::min_element(S_array.begin(), S_array.end());
    for (int i = 0; i < K; i++)
    {
        weight[i] = std::exp(-(S_array[i] - S_ref) / iota);
    }
    float sum_weight = 0;
    for (int i = 0; i < K; i++) sum_weight += weight[i];
    std::vector<float> weight_normal(K);
    for (int i = 0; i < K; i++) weight_normal[i] = weight[i] / sum_weight;
    // calc
    std::vector<float> sum_data(3, 0);
    std::vector<float> input_array(3, 0);
    for (int i = 0; i < 3; i++) for (int j = 0; j < K; j++) sum_data[i] += weight_normal[j] * all_v_array[j][0][i];
    for (int i = 0; i < 3; i++) input_array[i] = sum_data[i];

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