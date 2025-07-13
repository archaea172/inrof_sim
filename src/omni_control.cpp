#include <string>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class OmniControler : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // constructor
    OmniControler()
    : rclcpp_lifecycle::LifecycleNode(std::string("omni_controler"))
    {
        
    };

    // desconstructor
    ~OmniControler()
    {
        RCLCPP_INFO(this->get_logger(), "finish");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
    {
        joint_pub = this->create_publisher<std_msgs::msg::Float32>(
            std::string("wheel_joint"), rclcpp::SystemDefaultsQoS()
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        // activate publisher
        joint_pub->on_activate();

        // create subscriber
        vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            std::string("cmd_vel"),
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniControler::vel_callback, this, _1)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // deactivate publisher
        joint_pub->on_deactivate();
        // reset subscriber
        vel_subscriber.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
    {
        // reset publisher
        joint_pub.reset();
        return CallbackReturn::SUCCESS;
    }

    void vel_callback(const geometry_msgs::msg::Twist rxdata) const
    {

    }

private:
    // define publisher
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr joint_pub;
    // define subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
};