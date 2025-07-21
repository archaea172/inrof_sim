#include "pursuit.hpp"
#include "rclcpp/rclcpp.hpp"

PursuitControler::CallbackReturn PursuitControler::on_configure(const rclcpp_lifecycle::State &state)
{
    for (int i = 0; i < 3; i++)
    {
        p[i] = 0;
        v[i] = 0;
    }
    
    v_ref[0] = 1;
    v_ref[1] = 0.4;
    
    // m/s
    goal_p.resize(3);
    goal_p[0] = -1.2;
    goal_p[1] = 0;
    goal_p[2] = 0;
    T = 300;
    K = 100;
    dt = 0.01; // s
    // add max
    max_value[0] = 1;
    max_value[1] = 1;
    max_value[2] = 0.3;

    // create publisher
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PursuitControler> node = std::make_shared<PursuitControler>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}