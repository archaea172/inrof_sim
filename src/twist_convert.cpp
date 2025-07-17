#include <string>
#include <chrono>
#include <memory>
#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class TwistConverter : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // constructor
    TwistConverter()
    : rclcpp_lifecycle::LifecycleNode(std::string("twist_converter"))
    {
        v.resize(3);
    }
    // desconstructor
    ~TwistConverter()
    {
        RCLCPP_INFO(this->get_logger(), "finish");
    }

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        for (size_t i = 0; i < 3; i++) v[i] = 0;
        theta = 0;

        robot_twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            std::string("cmd_vel_robot"), rclcpp::SystemDefaultsQoS()
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        // activate publisher
        robot_twist_publisher->on_activate();

        // create subscriber
        controler_twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            std::string("cmd_vel"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TwistConverter::vel_callback, this, _1)
        );
        robot_pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
            std::string("odometry"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TwistConverter::pose_callback, this, _1)
        );

        pub_timer = this->create_wall_timer(0.01s, std::bind(&TwistConverter::timer_callback, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // deactivate publisher
        robot_twist_publisher->on_deactivate();
        // reset subscriber
        controler_twist_subscriber.reset();
        pub_timer.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        // reset publisher
        robot_twist_publisher.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "on error!", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return CallbackReturn::SUCCESS;
    }

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata)
    {
        this->v[0] = rxdata->linear.x;
        this->v[1] = rxdata->linear.y;
        this->v[2] = rxdata->angular.z;
    }

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
    {
        this->theta = rxdata->theta;
    }

    void timer_callback()
    {
        float rotation_theta = this->theta + M_PI/4;
        float vx_robot = this->v[0]*std::cos(rotation_theta) - this->v[1]*std::sin(rotation_theta);
        float vy_robot = this->v[0]*std::sin(rotation_theta) + this->v[1]*std::cos(rotation_theta);

        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = vx_robot;
        txdata.linear.y = vy_robot;
        txdata.angular.z = this->v[3];
        if (robot_twist_publisher->is_activated())
        {
            robot_twist_publisher->publish(txdata);
        }
    }

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr robot_twist_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controler_twist_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr robot_pose_subscriber;
    rclcpp::TimerBase::SharedPtr pub_timer;

    float theta;
    std::vector<float> v;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TwistConverter> node = std::make_shared<TwistConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}